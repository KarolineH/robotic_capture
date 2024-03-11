
import json
import numpy as np
from kortex_api.autogen.messages import Base_pb2

def read_action_from_file(filename):
    '''
    Reads a pose/angular/sequence action from a json file.
    Returns the action or sequence in robot-readable format.
    '''
    f = open(filename)
    data = json.load(f)
    label = next(iter(data))
    if label == 'poses':
        action = format_pose_action(data['poses']['pose'][0]['reachPose'])
    elif label == 'jointAnglesGroup':
        action = format_jointangle_action(data['jointAnglesGroup']['jointAngles'][0]['reachJointAngles'])
    elif label == 'sequences':
        sequence, action_list = format_sequence(data)
        return sequence, action_list
    else:
        print('Error: data type not recognized. Please check the file format.')
        return
    return action

def format_pose_action(data):
    '''
    Creates a cartesian action using data read from a json file.
    '''
    pose_data = data['targetPose']
    pose = np.array([pose_data['x'], pose_data['y'], pose_data['z'], pose_data['thetaX'], pose_data['thetaY'], pose_data['thetaZ']])
    action = create_cartesian_action(pose)
    return action

def format_jointangle_action(data):
    '''
    Creates a joint angle action using data read from a json file.
    '''
    joint_data = data['jointAngles']['jointAngles']
    joint_data = sorted(joint_data, key=lambda x: x["jointIdentifier"])
    angles = [item["value"] for item in joint_data]
    action = create_angular_action(angles)
    return action

def format_sequence(data):
    '''
    Creates a sequence using data read from a json file.
    '''
    sequence_data = data['sequences']['sequence'][0]['tasks']
    length = len(sequence_data)
    action_list = []
    for task in sequence_data:
        if next(iter(task['action'])) == 'reachPose':
            action = format_pose_action(task['action']['reachPose'])
            action_list.append(action)
        elif next(iter(task['action'])) == 'applicationData':
            action = format_jointangle_action(task['action']['reachJointAngles'])
            action_list.append(action)
        else:
            print('Error: action data type not recognized. Skipping action in sequence.')
            continue
    sequence = create_sequence(action_list)
    return sequence, action_list

def create_angular_action(joint_angles=None, delta=None, base_cyclic=None):
    '''
    Creates an angular action to be executed by the robot.
    Provide either a set of target joint angles or a set of joint angle deltas, 
    starting from the CURRENT pose at time of creation.
    If both are provided, the joint angles will be used.
    Input is a list or np.array of 6 values: [j1, j2, j3, j4, j5, j6]
    Returns the action.
    '''
    #TODO: Add support for creating a delta action from a specified previous state.
    actuator_count = 6 # We use a 6 DoF robot

    print("Creating angular action")
    action = Base_pb2.Action()
    action.name = "Angular action"
    action.application_data = ""

    if joint_angles is not None:
        if len(joint_angles) != actuator_count:
            print("Warning: The number of provided joint angles does not match the robot joints.")

    elif delta is not None and base_cyclic is not None:
        if len(delta) != actuator_count:
            print("Warning: The number of provided joint angle deltas does not match the robot joints.")
        current_angles = get_joint_angles(base_cyclic)
        joint_angles = current_angles + delta
    
    else:
        print("Warning: No joint angles or joint deltas provided")
        return
    
    for joint_id in range(actuator_count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = joint_angles[joint_id]
    return action

def create_cartesian_action(pose=None, delta=None, base_cyclic=None):

    '''
    Creates a cartesian action to be executed by the robot.
    Provide either a target pose or a pose delta, starting from the current pose.
    If both are provided, the target pose will be used.
    Input is a list or np.array of 6 values: [x, y, z, theta_x, theta_y, theta_z]
    Returns the action.
    '''

    print("Creating Cartesian action")
    action = Base_pb2.Action()
    action.name = "Cartesian action"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose
    if pose is not None:
        delta = pose
        starting_pose = np.array([0,0,0,0,0,0])
        #TODO: Check if zeroes is the correct default
    elif delta is not None and base_cyclic is not None:
        starting_pose = get_pose(base_cyclic)
    else:
        print("Warning: No pose or delta provided")
        return
    
    cartesian_pose.x = starting_pose[0] + delta[0]    # (meters)
    cartesian_pose.y = starting_pose[1] + delta[1]    # (meters)
    cartesian_pose.z = starting_pose[2] + delta[2]    # (meters)
    cartesian_pose.theta_x = starting_pose[3] + delta[3] # (degrees)
    cartesian_pose.theta_y = starting_pose[4] + delta[4] # (degrees)
    cartesian_pose.theta_z = starting_pose[5] + delta[5] # (degrees)
    return action

def create_sequence(actions):
    '''
    String multiple actions together into a sequence.
    Input is a list of actions.
    Returns the sequence.
    
    It is recommended to use the create_angular_action and create_cartesian_action 
    to create the actions.
    Also note that if you use deltas to create your actions, 
    they will NOT be relative to the previous action in this sequence. (TODO)
    '''

    sequence = Base_pb2.Sequence()
    sequence.name = "Sequence"

    for count, action in enumerate(actions):
        task = sequence.tasks.add()
        task.group_identifier = count
        task.action.CopyFrom(action)
    return sequence

def get_pose(base_cyclic):
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

def get_joint_angles(base_cyclic):
    '''
    Retrieve the current joint angles of the robot.
    Returns an np array of 6 values: [j1, j2, j3, j4, j5, j6] (degrees)
    '''
    feedback = base_cyclic.RefreshFeedback()
    joint_angles = np.asarray([joint.position for joint in feedback.actuators])
    return joint_angles

def state_to_pose(angles, base):
    '''
    Convert joint angles to cartesian pose
    '''
    pose = base.ComputeForwardKinematics(angles)
    return pose