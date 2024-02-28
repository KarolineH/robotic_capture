#!/usr/bin/env python3.8
import threading
import numpy as np
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# Local imports
try:
    # Attempt a relative import
    from .helpers import kortex_util as k_util # if being run as a package
    from .helpers import data_io
except ImportError:
    import helpers.kortex_util as k_util # local case
    import helpers.data_io as data_io

class Kinova3:
    def __init__(self):
        self.TIMEOUT_DURATION = 35 # Maximum allowed waiting time during actions (in seconds)

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

    # Create closure to set an event after an END or an ABORT
    def check_for_sequence_end_or_abort(self, e):
        """Return a closure checking for END or ABORT notifications on a sequence

        Arguments:
        e -- event to signal when the action is completed
            (will be set when an END or ABORT occurs)
        """

        def check(notification, e = e):
            event_id = notification.event_identifier
            task_id = notification.task_index
            if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
                print("Sequence task {} completed".format(task_id))
            elif event_id == Base_pb2.SEQUENCE_ABORTED:
                print("Sequence aborted with error {}:{}"\
                    .format(\
                        notification.abort_details,\
                        Base_pb2.SubErrorCodes.Name(notification.abort_details)))
                e.set()
            elif event_id == Base_pb2.SEQUENCE_COMPLETED:
                print("Sequence completed.")
                e.set()
        return check

    def move_to_home_position(self, base):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            return False

        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        base.ExecuteActionFromReference(action_handle)
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Safe position reached")
        else:
            print("Timeout on action notification wait")
        return finished

    def execute_sequence(self, base, sequence):

        e = threading.Event()
        notification_handle = base.OnNotificationSequenceInfoTopic(
            self.check_for_sequence_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Creating sequence on device and executing it")
        handle_sequence = base.CreateSequence(sequence)
        base.PlaySequence(handle_sequence)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if not finished:
            print("Timeout on action notification wait")
        return finished

    def execute_action(self, base, action):

        '''
        Executes an action on the robot.
        Input is the action to be executed.
        Returns True if the action was completed successfully, False otherwise.
        '''
        
        print("Preparing action movement ...")
        e = threading.Event()
        notification_handle = base.OnNotificationActionTopic(
            self.check_for_end_or_abort(e),
            Base_pb2.NotificationOptions()
        )

        print("Executing action")
        base.ExecuteAction(action)

        print("Waiting for movement to finish ...")
        finished = e.wait(self.TIMEOUT_DURATION)
        base.Unsubscribe(notification_handle)

        if finished:
            print("Movement completed")
        else:
            print("Timeout on action notification wait")
        return finished

    def get_pose(self, base_cyclic):
        '''
        Retrieve the current pose of the robot.
        Returns an np array of 6 values: [x, y, z, theta_x, theta_y, theta_z]
        '''
        feedback = base_cyclic.RefreshFeedback()
        position = np.array([feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z])
        orientation = np.array([feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z])
        # append together
        pose = np.append(position, orientation)
        return pose

    def get_joint_angles(self, base_cyclic):
        '''
        Retrieve the current joint angles of the robot.
        Returns an np array of 6 values: [j1, j2, j3, j4, j5, j6] (degrees)
        '''
        feedback = base_cyclic.RefreshFeedback()
        joint_angles = np.asarray([joint.position for joint in feedback.actuators])
        return joint_angles



def main():
    IF = Kinova3()
    # Parse arguments
    args = k_util.parseConnectionArguments()
    # Create connection to the device and get the router
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        # Example core
        success = True

        # ALL ACTIONS SHOULD GO HERE, WITHIN THE 'WITH' STATEMENT
        # so that the connection is closed properly afterwards
        success &= IF.move_to_home_position(base)
        example_sequence,__ = data_io.read_action_from_file("./data/DSLR_example_path.json")
        success &= IF.execute_sequence(base, example_sequence)
        # MOVE BACK into a stable position before powering off
        # be aware of your surroundings. The path is not always safe!
        rest_action = data_io.read_action_from_file("./data/rest_on_foam_cushion.json")
        success &= IF.execute_action(base, rest_action)

        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
