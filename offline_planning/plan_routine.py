import rclpy
from ros2_IK_client import IKClient

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import kinova_arm_if.helpers.conversion as conv
import numpy as np
from kinova_arm_if.helpers import conversion as conv
from calibration.helpers import plotting
import util 

# t_wrist_to_cam = conv.robot_poses_as_htms(np.array([-0.00034790886457611943, 0.04226343540493114, 0.08396112164677054, np.degrees(-1.581624895332124), np.degrees(-0.003688590448812868), np.degrees(-0.000676321102353462)])) #[x,y,z,theta_x,theta_y,theta_z] (in degrees)
# t_cam_to_wrist = conv.invert_transform(t_wrist_to_cam[0])
# space_limits = np.array([[-1, -1, -0.6], [1, 1, 1.3]])  # Cartesian space limits

def fibonacci_sphere(nr_samples):
    """
    Generate points on a sphere using the Fibonacci lattice.
    Note that this method is deterministic.
    """
    points = []
    phi = np.pi * (3. - np.sqrt(5.))  # golden angle in radians
    for i in range(nr_samples):
        y = 1 - (i / float(nr_samples - 1)) * 2  # y goes from 1 to -1
        radius = np.sqrt(1 - y * y)  # radius at y
        theta = phi * i  # golden angle increment
        x = np.cos(theta) * radius
        z = np.sin(theta) * radius
        points.append((x, y, z))
    return np.asarray(points)

def look_at_matrix(eyes):
    """
    Generate a look-at transformation matrix for a camera/eye pointing at the coordinate frame origin (0,0,0).
    """
    forward = - eyes / np.linalg.norm(eyes, axis=1).reshape((-1, 1))
    global_up = np.array([0, 0, -1])
    # if the forward direction is parallel with the global up vector, we need to choose a different up vector
    if np.allclose(forward, global_up) or np.allclose(forward, -global_up):
        global_up = np.array([0, 1, 0])

    right = np.cross(global_up, forward)
    right /= np.linalg.norm(right, axis=1).reshape((-1, 1))
    #right = np.divide(right, np.linalg.norm(right, axis=1).reshape((-1, 1)))

    up = np.cross(forward, right)
    mat = np.eye(4)
    mats = np.repeat(mat[np.newaxis, :, :], eyes.shape[0], axis=0)
    mats[:, :3, 0] = right
    mats[:, :3, 1] = up
    mats[:, :3, 2] = forward
    mats[:, :3, 3] = eyes
    return mats

# def transform_cam_to_wrist_frame(cam_poses):
#     """Transform a camera pose to the wrist frame."""
#     t_wrist = np.matmul(cam_poses, t_cam_to_wrist)
#     return t_wrist

def eucl_dist_from_j1(points):
    '''
    Computes the euclidean distance of a cartesian point from joint 1 of the robot.
    This is a fast exclusion criterion for far-away points, where IK solutions are impossible.
    '''
    j1 = [0,0,0.28]
    shifted_points = points - j1
    distances = np.linalg.norm(shifted_points, axis=1)
    return distances

def rotate_about_y(t, rad):
    """Rotate a transformation matrix about its local y-axis."""
    previous = t[:3,:3]
    r = np.array([
        [np.cos(rad), 0, np.sin(rad)],
        [0, 1, 0],
        [-np.sin(rad), 0, np.cos(rad)]
    ])
    new = np.matmul(previous, r)
    new_t = t.copy()
    new_t[:3,:3] = new
    return new_t

def compute_reachables(origins, radii, nr_of_queried_points=100, rotation_increment=0.2):
    '''
    Checks a number of reachable poses for the camera, given a set of origins and radii.

    origins: Nx3 array of origin locations
    radii: Mx1 array of radii
    nr_of_queried_points: number of points to query on the unit sphere
    rotation_increment: increment (in radians) for rotating the camera pose about its local y-axis to find more reachable poses
    '''
    # initialize the ROS2 Inverse Kinematics client
    rclpy.init()
    ik_client = IKClient()
    # Get poses on the unit sphere, pointing inward, as a starting point
    unit_points = fibonacci_sphere(nr_samples=nr_of_queried_points)
    unit_transforms = look_at_matrix(unit_points)
    steps = int(2*np.pi/rotation_increment)
    roll_angles = np.linspace(0, 2*np.pi, steps)

    i = 0
    results = []
    for radius in radii: # for each radius
        for origin in origins: # at each origin location
            i += 1
            print('Checking sphere {} of {}'.format(i, len(origins)*len(radii)))
            print('Origin:', origin, 'Radius:', radius)
            points = np.asarray(unit_points) * radius + origin # points on one of the potential spheres
            dists = eucl_dist_from_j1(points)
            idcs = np.where(dists < 1) # the robot's max reach is about 1m, discard points that are too far away
            points = points[idcs]

            tfs = unit_transforms[idcs]
            tfs[:, :3, 3] = points # these transforms are query poses for the camera (camera pose given in world frame)

            # Now check for IK solutions for each of the query poses
            joint_states = []
            reachable_poses = []
            for j,query in enumerate(tfs):
                for angle in roll_angles[:-1]:
                    query_rotated = rotate_about_y(query, angle)
                    quat = conv.mat_to_quat(query[:3, :3]) #(x, y, z, w)
                    error_code, angles = ik_client.send_request(*query[:3, 3], *quat, link='dslr_body_link') # poses are queried for the camera link
                    if error_code == -31:
                        #print("No IK solution, rejecting point.")
                        pass
                    elif error_code == 1:
                        # accepted
                        joint_states.append(angles.tolist())
                        reachable_poses.append(query_rotated)
                        break # no need to test more rotations, at least one reachable is found
                    else:
                        # catch potentially occurring unknown errors
                        print("Error code:", error_code)

            results.append((origin, radius, joint_states, reachable_poses))
            util.append_to_log('offline_planning/ik_log_90.txt', [origin.tolist(), radius, [entry for entry in joint_states], [entry.tolist() for entry in reachable_poses]])
    return results


def explore_reachability(recompute=False):
    if recompute:
        origins = np.array([[y, 0, z] for y in np.linspace(0,1,21) for z in np.linspace(0,1.2,13)])
        radii = np.array(np.linspace(0.7,1,7))#np.linspace(0.7)
        nr_of_queried_points = 100
        results = compute_reachables(origins, radii, nr_of_queried_points, rotation_increment=0.35)
    
    results = util.load_log('offline_planning/ik_log.txt')
    
    util.plot_radius_vs_reachables(results)
    util.plot_yz_offset_vs_reachables(results)
    util.plot_yzr_fit(results)
    return

def get_states(origin=[0,0.4,0.8], radius=0.7, n=1000):

    origin = np.asarray([origin])
    radius = np.asarray([radius])
    results = compute_reachables(origin, radius, n, rotation_increment=0.2)
    # results have the format (origin, radius, indeces of IK-reachable poses, joint_states, poses)

    cam_poses = np.asarray([res for res in results[0][4]])

    #plotting.plot_transforms(cam_poses)
    # util.plot_points(cam_poses[:, :3, 3])
    # util.save_pickle('offline_planning/04-08-07.pkl', results)

    # filter out any poses that would dip the camera below 10cm z (height), because of the table obstructing the workspace 
    # # Sort joint states to minimize total changes between them
    # joint_states = np.array(joint_states)
    # dist_matrix = euclidean_distances(joint_states, joint_states)
    # sorted_indices = [0]
    # for _ in range(len(joint_states) - 1):
    #     last_index = sorted_indices[-1]
    #     next_index = np.argmin(dist_matrix[last_index])
    #     dist_matrix[:, last_index] = np.inf  # Avoid revisiting
    #     sorted_indices.append(next_index)

    # sorted_joint_states = joint_states[sorted_indices]
    # return sorted_joint_states



if __name__ == '__main__':
    #results = util.load_log('offline_planning/ik_log.txt')
    #get_states()
    explore_reachability(recompute=True)

    # results = util.load_pickle('offline_planning/04-08-07.pkl')
    # wrist_poses = np.asarray([res for res in results[0][4]])
    # cam_poses = np.matmul(wrist_poses, t_wrist_to_cam[0])
    # util.plot_points(cam_poses[:, :3, 3])
