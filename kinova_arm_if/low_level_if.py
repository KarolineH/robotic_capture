
import argparse
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2
from kortex_api.autogen.messages import BaseCyclic_pb2

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

        # Set base in low level servoing mode
        servoing_mode_info = Base_pb2.ServoingModeInformation()
        servoing_mode_info.servoing_mode = Base_pb2.LOW_LEVEL_SERVOING
        self.base.SetServoingMode(servoing_mode_info)

        # Fetch first feedback
        base_feedback = self.base_cyclic.RefreshFeedback()

        # Create base cyclic command object.
        self.base_command = BaseCyclic_pb2.Command()
        self.base_command.frame_id = 0

        #TODO Confirm: Does this set the angles to 0? Or is it a delta from the current position?
        for actuator in base_feedback.actuators:
            self.actuator_command = self.base_command.actuators.add()
            self.actuator_command.position = actuator.position
            self.actuator_command.velocity = 0.0
            self.actuator_command.torque_joint = 0.0
            self.actuator_command.command_id = 0
            print("Position = ", actuator.position)

    def Goto(self, target_position):

        # This function is used to move the first joint
        # translates a position goal into velocity and runs until the goal is reached 

        while True:
            try:
                base_feedback = self.base_cyclic.Refresh(self.base_command)

                # Calculate speed according to position error (target position VS current position)
                position_error = target_position - base_feedback.actuators[0].position

                # If positional error is small, stop
                if abs(position_error) < 1.5:
                    position_error = 0
                    self.command.velocity = 0
                    self.base_cyclic.Refresh(self.base_command)
                    return True
                else:
                    self.command.velocity = self.proportional_gain * abs(position_error)
                    if self.command.velocity > 100.0:
                        self.command.velocity = 100.0
                    self.command.position = target_position

            except Exception as e:
                print("Error in refresh: " + str(e))
                return False
            time.sleep(0.001)
        return True

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--proportional_gain", type=float, help="proportional gain used in control loop", default=2.0)
    args = k_util.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        with k_util.DeviceConnection.createUdpConnection(args) as router_real_time:
            kbhit = kbhit.KBHit()
            example = LL_example(router, router_real_time, args.proportional_gain)
            example.Goto(90.0)


if __name__ == "__main__":
    main()
