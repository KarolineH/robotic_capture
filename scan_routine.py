#!/usr/bin/env python3.8

import os
import time
import pathlib

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
    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is enabled, you can instead add a focus operation before each capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, transform_path='./garbage.json')
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        #TODO: Add more poses to the sequence, vary the viewpoints more
        sequence, action_list = data_io.read_action_from_file(actions_dir + "/scan_path.json")
        for i,state in enumerate(action_list[:1]):
            IF.execute_action(state) # the first 3 poses are just for the robot to reach the starting position

        poses = []
        for i,state in enumerate(action_list[1:-1]):
            IF.execute_action(state)
            time.sleep(sleep_time) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
            cam_pose = IF.get_pose() # [x, y, z, theta_x, theta_y, theta_z]
            # TODO : Instead of waiting for a set duration, could check the wrist pose a few times to make sure it has stabilised
            poses.append(cam_pose)
            #cam.trigger_AF() # trigger the camera to focus
            path, cam_path, msg = cam.capture_image(download=True, target_path=out_dir) # capture an image and download it to the specified directory
        
        IF.execute_action(action_list[-1]) # the last pose is just for the robot to reach back to the home position

    pose_data = [pose.tolist() for pose in poses]
    poses_to_txt(pose_data, os.path.join(out_dir, 'cam_poses.txt'))
    return

def poses_to_txt(pose_data, path):
    translations = [pose[:3] for pose in pose_data]
    quaternions = [conv.euler_to_quat(*pose[3:]) for pose in pose_data]

    with open(path, 'w') as f:
        f.write("# QW, QX, QY, QZ, TX, TY, TZ\n")
        for i in range(len(pose_data)):
            f.write(f"{quaternions[i][-1]}, {quaternions[i][0]}, {quaternions[i][1]}, {quaternions[i][2]}, {translations[i][0]}, {translations[i][1]}, {translations[i][2]}\n")

if __name__ == "__main__":
    output_directory = '/home/kh790/data/test_scan'
    record_data(output_directory, sleep_time=5)