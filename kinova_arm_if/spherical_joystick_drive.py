#! /usr/bin/env python3

# Use joystick inputs to move the camera (tool) across the surface of a sphere
# 
# 1- Connection with the base:
#     1- A TCP session is started on port 10000 for most API calls. Refresh is at 25ms on this port.
#     2- A UDP session is started on port 10001 for BaseCyclic calls. Refresh is at 1ms on this port only.
# 2- Initialization
#     1- First frame is built based on arm feedback to ensure continuity
#     2- First actuator torque command is set as well
#     3- Base is set in low-level servoing
#     4- First frame is sent
#     3- First actuator is switched to torque mode
# 3- Cyclic thread is running at 1ms
#     1- Torque command to first actuator is set to a multiple of last actuator torque measure minus its initial value to
#        avoid an initial offset error
#     2- Position command to last actuator equals first actuator position minus initial delta
#     
# 4- On keyboard interrupt, example stops
#     1- Cyclic thread is stopped
#     2- First actuator is set back to position control
#     3- Base is set in single level servoing (default)
###

import sys
import os

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ActuatorCyclicClientRpc import ActuatorCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Session_pb2, ActuatorConfig_pb2, Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.RouterClient import RouterClientSendOptions

import time
import sys
import threading

class SphereTraverser:
    def __init__(self, router, router_real_time):

        # Maximum allowed waiting time during actions (in seconds)
        self.ACTION_TIMEOUT_DURATION = 20

        # Create required services
        self.router = router
        self.router_real_time = router_real_time
        device_manager = DeviceManagerClient(router)
        self.base = BaseClient(router)
        self.base_cyclic = BaseCyclicClient(router_real_time)

        self.base_command = BaseCyclic_pb2.Command()
        self.base_feedback = BaseCyclic_pb2.Feedback()
        self.actuator_config = ActuatorConfigClient(router)

        # Detect all devices
        device_handles = device_manager.ReadAllDevices()
        self.actuator_count = self.base.GetActuatorCount().count

        # Only actuators are relevant for this example
        for handle in device_handles.device_handle:
            if handle.device_type == Common_pb2.BIG_ACTUATOR or handle.device_type == Common_pb2.SMALL_ACTUATOR:
                self.base_command.actuators.add()
                self.base_feedback.actuators.add()

        # Change send option to reduce max timeout at 3ms
        self.sendOption = RouterClientSendOptions()
        self.sendOption.andForget = False
        self.sendOption.delay_ms = 0
        self.sendOption.timeout_ms = 3

        self.cyclic_t_end = 10  #Total duration of the thread in seconds. 0 means infinite.
        self.cyclic_thread = {}

        self.kill_the_thread = False
        self.already_stopped = False
        self.cyclic_running = False

    # Create closure to set an event after an END or an ABORT
    def check_for_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """
        def check(notification, e = e):
            print("EVENT : " + \
                Base_pb2.ActionEvent.Name(notification.action_event))
            if notification.action_event == Base_pb2.ACTION_END \
            or notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def MoveToHomePosition(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
    
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = self.base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        self.base.ExecuteActionFromReference(action_handle)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.ACTION_TIMEOUT_DURATION)
        self.base.Unsubscribe(notification_handle)

        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

        return True

    def InitCyclic(self, sampling_time_cyclic, t_end, print_stats):

        if self.cyclic_running:
            return True

        # Optionally move to Home position first
        # if not self.MoveToHomePosition():
        #     return False

        print("Init Cyclic")
        sys.stdout.flush()

        base_feedback = self.SendCallWithRetry(self.base_cyclic.RefreshFeedback, 3)
        if base_feedback:
            self.base_feedback = base_feedback

            # Init command frame
            for x in range(self.actuator_count):
                self.base_command.actuators[x].flags = 1 # servoing
                self.base_command.actuators[x].position = self.base_feedback.actuators[x].position
                # self.base_command.actuators[x].velocity = 0.0

            # Set arm in LOW_LEVEL_SERVOING
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
            time.sleep(0.5)
            self.base.SetServoingMode(base_servo_mode)
            
            # Send first frame
            self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)

            control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
            control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value('POSITION') # 1 POSITION, 2 VELOCITY

            for i in range(6):
                self.SendCallWithRetry(self.actuator_config.SetControlMode, 3, control_mode_message, i+1) # set each actuator to the same control mode

            # Init cyclic thread
            self.cyclic_t_end = t_end
            self.cyclic_thread = threading.Thread(target=self.RunCyclic, args=(sampling_time_cyclic, print_stats))
            self.cyclic_thread.daemon = True
            self.cyclic_thread.start()
            return True

        else:
            print("InitCyclic: failed to communicate")
            return False

    def RunCyclic(self, t_sample, print_stats):
        self.cyclic_running = True
        print("Run Cyclic")
        sys.stdout.flush()
        cyclic_count = 0  # Counts refresh
        stats_count = 0  # Counts stats prints
        failed_cyclic_count = 0  # Count communication timeouts

        initial_state = [actuator.position for actuator in self.base_feedback.actuators]
        state = initial_state

        t_now = time.time()
        t_cyclic = t_now  # cyclic time
        t_stats = t_now  # print  time
        t_init = t_now  # init   time

        durations = []
        print("Running example for {} seconds".format(self.cyclic_t_end))
        while not self.kill_the_thread:
            t_now = time.time()
            durations.append(t_now - t_cyclic)

            # Cyclic Refresh
            if (t_now - t_cyclic) >= t_sample:
                t_cyclic = t_now

                # TODO: The main meat goes here
                # check the input (button press, joystick, torque...)
                # then compute the desired cartesian pose
                # then translate to desired joint state
                # when using position control, the joint skips(?) or abruptly stops every now and then
                # (could also be a joint velocity command, but when I use velocity control, the camera drifts down as if the breaks were not applied, regardless of robot pose)
                # This is a KNOWN and UNSOLVED problem, basically, low level velocity control is not working, the company recommends using position control!????
                # possibly chose one of multiple possible solutions
                # ...
                # state[-1] = state[-1] - 0.0001* cyclic_count
                
                self.base_command.actuators[-1].position = self.base_feedback.actuators[-1].position - 0.001* cyclic_count

                # Incrementing identifier ensure actuators can reject out of time frames
                self.base_command.frame_id += 1
                if self.base_command.frame_id > 65535:
                    self.base_command.frame_id = 0
                for i in range(self.actuator_count):
                    self.base_command.actuators[i].command_id = self.base_command.frame_id

                # Frame is sent
                try:
                    self.base_feedback = self.base_cyclic.Refresh(self.base_command, 0, self.sendOption)
                except Exception as e:
                    failed_cyclic_count = failed_cyclic_count + 1
                    print("Failed a count")
                    print(e)
                cyclic_count = cyclic_count + 1

            # Stats Print
            if print_stats and ((t_now - t_stats) > 1):
                t_stats = t_now
                stats_count = stats_count + 1
                
                cyclic_count = 0
                failed_cyclic_count = 0
                sys.stdout.flush()

            if self.cyclic_t_end != 0 and (t_now - t_init > self.cyclic_t_end):
                print("Cyclic Finished")
                sys.stdout.flush()
                break
        self.cyclic_running = False
        return True

    def StopCyclic(self):
        print ("Stopping the cyclic and putting the arm back in position mode...")
        if self.already_stopped:
            return

        # Kill the  thread first
        if self.cyclic_running:
            self.kill_the_thread = True
            self.cyclic_thread.join()
        
        # Set all actuators back in position mode
        control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
        control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value('POSITION') # 1 POSITION, 2 VELOCITY
        for i in range(6):
            self.SendCallWithRetry(self.actuator_config.SetControlMode, 3, control_mode_message, i+1) # set each actuator to the same control mode 
        device_id = 1  # first actuator has id = 1
        
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        self.cyclic_t_end = 0.1

        self.already_stopped = True
        
        print('Clean Exit')

    @staticmethod
    def SendCallWithRetry(call, retry,  *args):
        i = 0
        arg_out = []
        while i < retry:
            try:
                arg_out = call(*args)
                break
            except:
                i = i + 1
                continue
        if i == retry:
            print("Failed to communicate")
        return arg_out

def main():
    # Import the utilities helper module
    import argparse
    try:
        from .helpers import kortex_util as utilities
    except ImportError:
        import helpers.kortex_util as utilities # local case

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--cyclic_time", type=float, help="delay, in seconds, between cylic control call", default=0.001)
    parser.add_argument("--duration", type=int, help="example duration, in seconds (0 means infinite)", default=30)
    parser.add_argument("--print_stats", default=True, help="print stats in command line or not (0 to disable)", type=lambda x: (str(x).lower() not in ['false', '0', 'no']))
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:

            mover = SphereTraverser(router, router_real_time)
            success = mover.InitCyclic(args.cyclic_time, args.duration, args.print_stats)

            if success:

                while mover.cyclic_running:
                    try:
                        time.sleep(0.5)
                    except KeyboardInterrupt:
                        break
            
                mover.StopCyclic()

            return 0 if success else 1


if __name__ == "__main__":
    exit(main())
