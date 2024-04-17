import os
import numpy as np
from helpers import plotting
from scipy.spatial.transform import Rotation


''' 
Plots the poses that were recorded off the robot arm during the scanning routine.
This is useful for debugging and verifying that the poses are correct.
The coordinate frame of the camera is a right-hand coordinate system with positive axes going x-right, y-down, z-forward (away from camera through the image plane). 
This is the same as the OpenCV convention.
This script also offers a method for converting from previously saved right-hand to left-hand poses.
'''



def plot(files):

    for path in files:
        # read data from text file
        poses = np.loadtxt(path, delimiter=',') # n x 7 array, [QW, QX, QY, QZ, TX, TY, TZ]

        # convert to 4x4 homogeneous transformation matrices
        num_poses = poses.shape[0]
        transforms = []
        for i in range(num_poses):
            # scipy reads quaternions in scalar-last (x, y, z, w) format
            # so we rearrange the quaternion from the file to match this format
            quaternion = poses[i, 1:4] # QY, QZ, QW
            quaternion = np.append(quaternion, poses[i, 0]) # QX

            translation = poses[i, 4:]
           # translations = [[x, y, -z] for x, y, z in translations]
           # euler_angles = [[x, y, -z] for x, y, z in euler_angles]
            transform = np.eye(4)
            transform[:3, -1] = poses[i, 4:]
            transforms.append(transform)


            rotation_matrix = Rotation.from_quat(quaternion).as_matrix()

            euler_angles = Rotation.from_quat(quaternion).as_euler('xyz', degrees=True)


            transform[:3, :3] = rotation_matrix



        plotting.plot_transforms(np.array(transforms))
    print('Done!')

def right_to_left_handed_frame(files):
    for path in files:
        updated_path = path[:-4] + '_left_handed.txt'
        poses = np.loadtxt(path, delimiter=',') # n x 7 array, [QW, QX, QY, QZ, TX, TY, TZ]
        num_poses = poses.shape[0]

        updated_poses = []
        for i in range(num_poses):
            # scipy reads quaternions in scalar-last (x, y, z, w) format
            # so we rearrange the quaternion from the file to match this format
            quaternion = poses[i, 1:4] # QY, QZ, QW
            quaternion = np.append(quaternion, poses[i, 0]) # QX
            translation = poses[i, 4:]
            euler_angles = Rotation.from_quat(quaternion).as_euler('xyz', degrees=True)

            # flip z axiz
            translation[-1] *= -1
            euler_angles[-1] *= -1

            # convert back to quaternion
            quaternion_left = Rotation.from_euler('xyz', euler_angles, degrees=True).as_quat()
            updated_pose = [quaternion_left[3], quaternion_left[0], quaternion_left[1], quaternion_left[2], translation[0], translation[1], translation[2]]
            updated_poses.append(updated_pose)
        with open(updated_path, 'w') as f:
            f.write("# QW, QX, QY, QZ, TX, TY, TZ\n")
            for line in updated_poses:
                f.write(f"{line[0]}, {line[1]}, {line[2]}, {line[3]}, {line[4]}, {line[5]}, {line[6]}\n")
    print('Done!')



files = ["/home/karo/Downloads/cam_poses.txt", "/home/karo/Downloads/cam_poses(1).txt", "/home/karo/Downloads/cam_poses(2).txt", "/home/karo/Downloads/cam_poses(3).txt"]
#right_to_left_handed_frame(files)
#files2 = ["/home/karo/Downloads/cam_poses_left_handed.txt", "/home/karo/Downloads/cam_poses(1)_left_handed.txt", "/home/karo/Downloads/cam_poses(2)_left_handed.txt", "/home/karo/Downloads/cam_poses(3)_left_handed.txt"]
plot(files)
