import kinova_arm_if.helpers.data_io as data_io
import pathlib
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.metrics.pairwise import euclidean_distances

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.Exceptions.KServerException import KServerException
from kinova_arm_if.helpers import conversion as conv
from calibration.helpers import plotting


def show_sample_states():
    import kinova_arm_if.helpers.kortex_util as k_util
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        base = BaseClient(router)

        # Read some manually approved sample states (joint angles) from a file
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        states = data_io.raw_state_sequence_from_file(actions_dir + "/orienting the robot's new workspace for caliibration.json")

        # Use forward kinematics to compute the cartesian pose of the end effector in each state
        poses = []
        for state in states:
            try:
                pose = base.ComputeForwardKinematics(states)
                poses.append(pose)
            except KServerException as ex:
                print("Error_code:{} , Sub_error_code:{} ".format(ex.get_error_code(), ex.get_error_sub_code()))
    return poses


def fibonacci_sphere(nr_samples):
    """Generate points on a sphere using the Fibonacci lattice.
    Note that this method is deterministic."""
    points = []
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians
    for i in range(nr_samples):
        y = 1 - (i / float(nr_samples - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)  # radius at y
        theta = phi * i  # golden angle increment
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        points.append((x, y, z))
    return points

def look_at_matrix(eye, target):
    """Generate a look-at transformation matrix for the camera."""
    forward = (target - eye) / np.linalg.norm(target - eye)
    up = np.array([0, 0, 1])
    right = np.cross(up, forward)
    right /= np.linalg.norm(right)
    up = np.cross(forward, right)
    mat = np.eye(4)
    mat[:3, 0] = right
    mat[:3, 1] = up
    mat[:3, 2] = forward
    mat[:3, 3] = eye
    return mat

def inverse_kinematics(point, angles):
    '''placeholder for the IK function'''
    joint_angles = np.zeros(6)
    return joint_angles

def generate_poses():
    origin = np.array([0, 0, 0])  # Origin of the sphere (e.g. object center)
    radii = np.array([1.0, 0.5])  # Radii for all spheres
    num_points = 500  # Number of points per radius

    space_limits_min = np.array([-1.1, -1.1, -1.1])  # Cartesian space limits
    space_limits_max = np.array([1.1, 1.1, 1.1])

    no_go_zones_min = np.array([-2, -0.2, -2], [-2, -0.2, -2])  # No-go zones, rectangular exclusion zones
    no_go_zones_max = np.array([-0, 0.2, 2], [-0, 0.2, 2])

    # Generate points on the sphere
    sphere_points = np.asarray(fibonacci_sphere(num_points))
    total_points = []
    for radius in radii:
        total_points.extend(sphere_points * radius)
    total_points = np.asarray(total_points)
    total_points += origin

    # Filter points based on Cartesian constraints
    valid_points = []
    for point in total_points:
        if np.any(point <= space_limits_min) or np.any(point >= space_limits_max):
            continue
        if np.all(point >= exclude_cube_min) and np.all(point <= exclude_cube_max):
            continue
        valid_points.append(point)
    valid_points = np.asarray(valid_points)

    # create a rotation to suit the points and convert into robot poses [x,y,z,theta_x,theta_y,theta_z]
    tfs = []
    for point in valid_points:
        pose = look_at_matrix(point, origin)
        tfs.append(pose)
    plotting.plot_transforms(np.asarray(tfs[:100]))
    thetas = [conv.mat_to_euler(pose[:3,:3]) for pose in tfs]
    # this next conversion (IK) is needed, and should also be able to make some judgement about reachability
    joint_states = [inverse_kinematics(point,angles) for point,angles in zip(valid_points,thetas)]

    # Sort joint states to minimize total changes between them
    joint_states = np.array(joint_states)
    dist_matrix = euclidean_distances(joint_states, joint_states)
    sorted_indices = [0]
    for _ in range(len(joint_states) - 1):
        last_index = sorted_indices[-1]
        next_index = np.argmin(dist_matrix[last_index])
        dist_matrix[:, last_index] = np.inf  # Avoid revisiting
        sorted_indices.append(next_index)

    sorted_joint_states = joint_states[sorted_indices]
    return sorted_joint_states

def plot_points(points):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_coords, y_coords, z_coords = zip(*points)
    ax.scatter(x_coords, y_coords, z_coords, s=1)

    # Set the aspect ratio to be equal
    ax.set_box_aspect([1,1,1])
    plt.show()
    return

if __name__ == '__main__':
    poses = generate_poses()
    print(poses)
    # poses = show_sample_states()
    # print(poses)