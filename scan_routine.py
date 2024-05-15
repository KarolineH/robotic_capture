#!/usr/bin/env python3.8

import os
import time
import pathlib
import datetime
import numpy as np

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv

''' 
Routine for taking captures intended for 3D reconstruction or novel view synthesis. The focus here lies on precision of the camera poses and the quality of the images.
This routine will record move the robot through a series of target states and record both the wrist poses and a full-resolution image in each state.
Note that pauses are added between the robot movements to allow for the robot to come to a complete stop before capturing the image.

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
Also ensure that the robot eye-in-hand calibration and calibration of camera intrinsics have been performed before running this routine.
'''

def record_data(out_dir, capture_params=[32,'AUTO','AUTO',True], sleep_time=2):

    # create new directory for the scan
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = os.path.join(out_dir, stamp)
    os.mkdir(im_dir)

    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is enabled, you can instead add a focus operation before each capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        sequence, action_list = data_io.read_action_from_file(actions_dir + "/calibration_sequence_44.json")
        for i,state in enumerate(action_list[:1]):
            IF.execute_action(state) # the first 3 poses are just for the robot to reach the starting position

        poses = []
        for i,state in enumerate(action_list[1:-1]):
            IF.execute_action(state)
            time.sleep(sleep_time) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
            cam_pose = IF.get_pose() # [x, y, z, theta_x, theta_y, theta_z]
            poses.append(cam_pose)
            #cam.trigger_AF() # trigger the camera to focus
            path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory
            #TODO: keep record of the file names linked with the corresponding poses!       

        IF.execute_action(action_list[-1]) # the last pose is just for the robot to reach back to the home position

    pose_data = [pose.tolist() for pose in poses]
    poses_to_txt(pose_data, os.path.join(im_dir, 'cam_poses.txt'))
    return

def poses_to_txt(pose_data, path):
    # First, save the raw poses to a separate file for future reference and debugging.
    raw_poses = np.asarray(pose_data)
    raw_file = '/'.join(path.split('/')[:-1]) + '/raw_poses.txt'
    np.savetxt(raw_file, raw_poses, delimiter=',', comments='x,y,z,theta_x,theta_y,theta_z')

    # 1. COLMAP expects the transformation from world to camera frame, not from camera to world frame. 
    # Need to invert the transformation. We will do this in homogeneous transformation matrix form.
    transforms = conv.robot_poses_as_htms(np.asarray(pose_data))
    inv_transforms = np.asarray([conv.invert_transform(mat) for mat in transforms]) # invert the transforms
    Rs = inv_transforms[:, :3, :3]
    ts = inv_transforms[:, :3, 3]

    # 2. COLMAP expects the camera poses in quaternion format, while the robot gives them in euler angles. Need to convert the euler angles to quaternions.
    quaternions = conv.mat_to_quat(Rs)
    # reorder the quaternion to scalar-first format
    quaternions = np.concatenate((quaternions[:,3].reshape(-1,1), quaternions[:,:3]), axis=1)

    # 3. Formatting and saving to txt file
    with open(path, 'w') as f:
        f.write("# id, QW, QX, QY, QZ, TX, TY, TZ, camera_id\n")
        for i in range(len(pose_data)):
            # COLMAP expects image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, (file name, TBD)
            #TODO: write the corresponding image file name to the end of each line
            f.write(f"{i+1} {quaternions[i][0]} {quaternions[i][1]} {quaternions[i][2]} {quaternions[i][3]} {ts[i][0]} {ts[i][1]} {ts[i][2]} 1 \n")
            f.write("\n") # add a newline between each pose, COLMAP expects this

if __name__ == "__main__":
    #output_directory = '/home/kh790/data/scans'
    #record_data(output_directory, sleep_time=5)

    # in_file = '/home/karo/Desktop/original_poses_before_conversion3.txt'
    # pose_data = np.loadtxt(in_file)
    # poses = poses_to_txt(pose_data, in_file)