import rclpy
from ros2_IK_client import IKClient

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import kinova_arm_if.helpers.conversion as conv
import numpy as np
from kinova_arm_if.helpers import conversion as conv
from calibration.helpers import plotting

t_wrist_to_cam = conv.robot_poses_as_htms(np.array([-0.00034790886457611943, 0.04226343540493114, 0.08396112164677054, np.degrees(-1.581624895332124), np.degrees(-0.003688590448812868), np.degrees(-0.000676321102353462)])) #[x,y,z,theta_x,theta_y,theta_z] (in degrees)
t_cam_to_wrist = conv.invert_transform(t_wrist_to_cam[0])
space_limits = np.array([[-1, -1, -0.6], [1, 1, 1.3]])  # Cartesian space limits

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
    
    return np.asarray(points)

def look_at_matrix(eyes):
    """
    Generate a look-at transformation matrix for a camera/eye pointing at a target.
    In this case, the target is the origin (0,0,0).
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

def transform_cam_to_wrist_frame(cam_poses):
    """Transform a camera pose to the wrist frame."""
    t_wrist = np.matmul(cam_poses, t_cam_to_wrist)
    return t_wrist

def eucl_dist_from_j1(points):
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

def compute_reachables(origins, radii, nr_of_queried_points=100):

    # initialize the ROS2 Inverse Kinematics client
    rclpy.init()
    ik_client = IKClient()
    unit_points = fibonacci_sphere(nr_samples=nr_of_queried_points)
    unit_transforms = look_at_matrix(unit_points)

    i = 0
    results = []
    for radius in radii:
        for origin in origins:
            i += 1
            print('Checking sphere {} of {}'.format(i, len(origins)*len(radii)))
            points = np.asarray(unit_points) * radius + origin # points on one of the potential spheres
            
            dists = eucl_dist_from_j1(points)
            idcs = np.where(dists < 1) # the robot's max reach is about 1m, discard points that are too far away
            points = points[idcs]

            tfs = unit_transforms[idcs]
            tfs[:, :3, 3] = points
            # tfs are now the query camera poses given in camera frame 
            # until here the poses look correct!

            # Transform the points to the wrist frame
            tfs_wrist = transform_cam_to_wrist_frame(tfs)
            #plotting.plot_transforms(tfs_wrist)

            reachable_points = []
            joint_states = []
            reachable_poses = []

            for j,query in enumerate(tfs_wrist):
                angles = np.linspace(0, 2*np.pi, 10)
                for angle in angles[:-1]:
                    query_rotated = rotate_about_y(query, angle)
                    quat = conv.mat_to_quat(query[:3, :3]) #(x, y, z, w)
                    error_code, angles = ik_client.send_request(*query[:3, 3], *quat)
                    if error_code == -31:
                        #print("No IK solution, rejecting point.")
                        pass
                    elif error_code == 1:
                        # accepted
                        reachable_points.append(j)
                        joint_states.append(angles)
                        reachable_poses.append(query_rotated)
                        break # no need to test more rotations, at least one reachable is found
                    else:
                        # catch potentially occurring unknown errors
                        print("Error code:", error_code)

            results.append((origin, radius, reachable_points, joint_states, reachable_poses))
    return results

def plot_points(points):
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_coords, y_coords, z_coords = zip(*points)
    ax.scatter(x_coords, y_coords, z_coords, s=20)

    # Set the aspect ratio to be equal
    ax.set_box_aspect([1,1,1])
    plt.show()
    return

def plot_radius_vs_reachables(results):
    import matplotlib
    matplotlib.use('Qt5Agg')
    matplotlib.pyplot.scatter(np.asarray([q[1] for q in results]),np.asarray([len(q[2]) for q in results]))
    matplotlib.pyplot.show()
    return

def plot_yz_offset_vs_reachables(results):
    import matplotlib
    matplotlib.use('Qt5Agg')
    # make a 2D heatmap where x and y are the offsets and the color is the number of reachable points
    x = np.asarray([q[0][1] for q in results])
    y = np.asarray([q[0][2] for q in results])
    z = np.asarray([len(q[2]) for q in results])
    matplotlib.pyplot.hist2d(x, y, bins=(16, 21), weights=z, cmap=matplotlib.pyplot.cm.jet)
    matplotlib.pyplot.colorbar()
    matplotlib.pyplot.xlabel('Y offset')
    matplotlib.pyplot.ylabel('Z offset')
    matplotlib.pyplot.show()
    return
    
def plot_yzr_fit(results):
    # 3D plot
    import matplotlib
    matplotlib.use('Qt5Agg')
    x = np.asarray([q[0][1] for q in results])
    y = np.asarray([q[0][2] for q in results])
    z = np.asarray([q[1] for q in results])
    c = np.asarray([len(q[2]) for q in results])
    fig = matplotlib.pyplot.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z, c=c, cmap='viridis')
    ax.set_xlabel('y')
    ax.set_ylabel('z')
    ax.set_zlabel('Radius')
    matplotlib.pyplot.show()

def save_pickle(file_name, results):
    import pickle
    with open(file_name, 'wb') as f:
        pickle.dump(results, f)

def load_pickle(file_name):
    import pickle
    with open(file_name, 'rb') as f:
        return pickle.load(f)


def explore_reachables():
    # Either compute reachable poses
    # origins = np.array([[0, 0.4, 0.8]])
    # radii =  np.array([0.7])
    origins = np.array([[0, y, z] for y in np.linspace(0,1,21) for z in np.linspace(0.2,1.5,27)])
    radii = np.array([0.7])#np.linspace(0.7)
    nr_of_queried_points = 100
    results = compute_reachables(origins, radii, nr_of_queried_points)
    #save_pickle('offline_planning/pose_exploration.pkl', results)

    # Or load earlier results from a pickle file
    # results = load_pickle('offline_planning/pose_exploration_2.pkl')
    plot_radius_vs_reachables(results)
    plot_yz_offset_vs_reachables(results)
    plot_yzr_fit(results)
    return

def get_states():
    origin = np.array([[0, 0.4, 0.8]])
    radius =  np.array([0.7])
    nr_of_queried_points = 1000

    results = compute_reachables(origin, radius, nr_of_queried_points)
    # results have the format (origin, radius, indeces of IK-reachable poses, joint_states, poses)
    # translate wrist poses back to camera poses
    wrist_poses = np.asarray([res for res in results[0][4]])
    cam_poses = np.matmul(wrist_poses, t_wrist_to_cam[0])
    #plotting.plot_transforms(cam_poses)
    plot_points(cam_poses[:, :3, 3])


    save_pickle('offline_planning/04-08-07.pkl', results)

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
    #get_states()
    #explore_reachables()

    results = load_pickle('offline_planning/04-08-07.pkl')
    wrist_poses = np.asarray([res for res in results[0][4]])
    cam_poses = np.matmul(wrist_poses, t_wrist_to_cam[0])
    plot_points(cam_poses[:, :3, 3])
