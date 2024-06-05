
import argparse
import time
import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, ActuatorConfig_pb2, BaseCyclic_pb2, ControlConfig_pb2
from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import ActuatorConfigClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import ControlConfigClient


# Local imports
try:
    # Attempt a relative import
    from .helpers import kortex_util as k_util # if being run as a package
    from .helpers import kbhit
except ImportError:
    import helpers.kortex_util as k_util # local case
    import helpers.kbhit as kbhit

# use Kinova.Api.BaseCyclic API
# to send commands up to 1kHz

# set the robot to the correct mode
# SetServoingMode(ServoingModeInformation)
# from base

# select the type of action 
# SetControlMode(ControlModeInformation)
# from ActuatorConfig client

# Refresh(Command)
# RefreshCommand()
# RefreshFeedback()
# can provide actuator position, angular velocity, torque, motor current


# When command types other than Joint positions are sent, always include the latest feedback joint position in the command
class LL_example:
    def __init__(self, router, router_real_time, proportional_gain = 2.0):
        self.router = router
        self.router_real_time = router_real_time
        self.base = BaseClient(self.router)
        self.base_cyclic = BaseCyclicClient(self.router_real_time)
        self.control_config = ControlConfigClient(router)
        self.previous_control_mode = self.control_config.GetControlMode()
        control_mode_info = ControlConfig_pb2.ControlModeInformation()
        control_mode_info.control_mode = ControlConfig_pb2.CARTESIAN_VELOCITY



        self.proportional_gain = proportional_gain

        # Set base in low level servoing mode
        self.previous_servoing_mode = self.base.GetServoingMode()
        servoing_mode_info = Base_pb2.ServoingModeInformation()
        servoing_mode_info.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)

        # Fetch first feedback
        base_feedback = self.base_cyclic.RefreshFeedback()

        # Create base cyclic command object.
        self.base_command = BaseCyclic_pb2.Command()
        self.base_command.frame_id = 0

        # Set the command to the current joint positions
        # Set the velocity and torque to 0
        for actuator in base_feedback.actuators:
            self.actuator_command = self.base_command.actuators.add()
            self.actuator_command.position = actuator.position
            self.actuator_command.velocity = 0.0
            self.actuator_command.torque_joint = 0.0
            self.actuator_command.command_id = 0
            self.actuator_command.flags = 0
            print("Position = ", actuator.position)

    def stop_all_joints(self):
        for actuator in self.base_command.actuators:
            actuator.velocity = 0.0
        self.base_cyclic.Refresh(self.base_command)

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

    def move_joints_delta(self, delta):
        # Provide a list of 6 elements, each element is the delta for the corresponding joint

        if len(delta) != 6:
            print("Delta must be a list of 6 elements") 
        base_feedback = self.base_cyclic.RefreshFeedback()
        start_positions = [joint.position for joint in base_feedback.actuators]
        target_positions = [start_positions[i] + delta[i] for i in range(6)]

        mm = ActuatorConfig_pb2.GetControlMode()
        ActuatorConfig_pb2.ControlModeInformation()

        while True:
            try:
                base_feedback = self.base_cyclic.RefreshFeedback()
                current_positions = [joint.position for joint in base_feedback.actuators]
                position_error = np.asarray([target_positions[i] - base_feedback.actuators[i].position for i in range(len(target_positions))])

                # If positional error is small, stop
                if max(abs(position_error)) < 1.5:
                    position_error = 0
                    self.stop_all_joints()
                    return True
                else:
                    for actuator, target, current, error in zip(self.base_command.actuators, target_positions, current_positions, position_error):
                        actuator.velocity = self.proportional_gain * abs(error) # abs?
                        actuator.position = current
                        if actuator.velocity > 100.0:
                            actuator.velocity = 100.0
                        if actuator.velocity > 0:
                            actuator.flags = 1
                    r = self.base_cyclic.Refresh(self.base_command)
                        # TODO make the velocity limits dependend on the joint nr    

            except Exception as e:
                print("Error in refresh: " + str(e))
                return False
            time.sleep(0.001)
        return True
    
    def Cleanup(self):
        # Restore servoing mode to the one that was in use before running the example
        self.base.SetServoingMode(self.previous_servoing_mode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--proportional_gain", type=float, help="proportional gain used in control loop", default=2.0)
    args = k_util.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        with k_util.DeviceConnection.createUdpConnection(args) as router_real_time:
            keyboard = kbhit.KBHit()
            example = LL_example(router, router_real_time, args.proportional_gain)
            example.move_joints_delta([0,-10,0,0,0,0])
            #example.Cleanup()


if __name__ == "__main__":
    main()
