#!/usr/bin/env python3.8

import os
import time
import json
import numpy as np
import cv2
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
import kinova_arm_if.helpers.conversion as conv
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io

''' 
Routine for calibrating the hand-eye coordination between robot arm and mounted DSLR camera.
This should only need to be performed once, and again whenever the camera is moved or the dimensions of the camera, mount, or lens change.

This routine will record move the robot through 10 target states and record both the wrist poses and a full-resolution image in each state.
In parallel, you can optionally record a 1080p video feed from the camera throughout the whole movement. 

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
The camera should be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''


output_directory = '/home/kh790/data/calibration_imgs/hand_eye_coord'
camera = '/dev/video2' # change this to the correct video device if needed


def record_data(remote_control_cam = False, record_video = False):

    # INPUT: set to True if you want to control the camera focus and shutter remotely, False if you want to snap the pictures by hand

    IF = Kinova3() # get robot Python interface object, has all needed methods
    if remote_control_cam:
        cam = EOS() # instantiate the camera interface object
        #cam.set_capture_parameters(aperture='AUTO', iso='AUTO', shutterspeed='AUTO', c_AF=False) # change capture settings if needed
        # for i in range(10):
        #     cam.manual_focus(2)
    if record_video:
        rec = Recorder(camera) # instantiate the HDMI video recorder object

    # Create connection to the robot and get the router
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        success = True

        # double check that the robot is in its safe resting position
        rest_action = data_io.read_action_from_file("/home/kh790/ws/robotic_capture/kinova_arm_if/data/rest_on_foam_cushion.json")
        success &= IF.execute_action(base, rest_action)

        example_sequence, action_list = data_io.read_action_from_file("/home/kh790/ws/robotic_capture/kinova_arm_if/data/hand_eye_sequence.json")
        poses = []

        if record_video:
            rec.start_recording(os.path.join(output_directory, 'video' ,'hand_eye_calibration.mp4')) # start recording the robot's movement

        for i,state in enumerate(action_list):
            IF.execute_action(base, state)
            if i > 1 and i < len(action_list)-1:
                time.sleep(2) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
                wrist_pose = IF.get_pose(base_cyclic)
                poses.append(wrist_pose)
                if remote_control_cam:
                    #cam.trigger_AF() # trigger the camera to focus
                    path, msg = cam.capture_image(download=True, target_path=output_directory) # capture an image and download it to the specified directory
                    print(msg)
        if record_video:           
            rec.stop_recording()

        pose_data = [pose.tolist() for pose in poses]
        json.dump(pose_data, open(os.path.join(output_directory, 'hand_eye_wrist_poses.json'), 'w'))


def get_camera_poses(output_directory, cam_calib_file = 'calibration.yaml'):
    cc = CamCalibration('eos_test', output_directory)

    # FIRST DO regular camera calibration
    calibration_img_dir = '/home/kh790/data/calibration_imgs/april_tags'
    res,mtx,dist,transforms, used = cc.april_tag_calibration(im_dir=calibration_img_dir)
    calibration_io.save_to_yaml('calibration.yaml', cc.name, res, mtx, dist)

    # THEN evaluate the images take in the hand-eye calibration routine
    cam_name, frame_size, matrix, distortion = calibration_io.load_from_yaml(cam_calib_file)
    __, __, __, cam_in_world, used = cc.april_tag_calibration(matrix, distortion)
    return cam_in_world, used

def get_wrists_poses(output_directory):
    with open(os.path.join(output_directory, 'hand_eye_wrist_poses.json'), 'r') as f:
        wrist_poses = json.load(f)
    return wrist_poses

def coordinate(cam_coords, wrist_coords):
    # TODO: Wrist poses are given as [x, y, z, theta_x, theta_y, theta_z]

    wrist_R = []
    wrist_t = []
    for pose in wrist_coords:
        pose = np.array(pose)
        tvec = pose[:3]
        euler_angles = pose[3:]
        rmat = conv.euler_to_mat(*euler_angles)
        wrist_R.append(rmat)
        wrist_t.append(tvec)
    
    wrist_R = np.array(wrist_R)
    wrist_t = np.array(wrist_t)

    # Camera poses are given as 4x4 homogeneous transformation matrices
    # I think these need to be inverted though, it calls for the inverse transformation
    cam_R = cam_coords[:,:3,:3]
    cam_t = cam_coords[:,:3,3]

    # cv2.calibrateHandEye() takes 4x4 matrices as input
    R_cam2grippe, t_cam2grippe = cv2.calibrateHandEye(wrist_R, wrist_t, cam_R, cam_t, method=cv2.CALIB_HAND_EYE_PARK)
    # there are also different methods available, see (HandEyeCalibrationMethod)

    # maybe use cv2.calibrateRobotWorldHandEye() instead, it spits out the pattern location as well.


    return

if __name__ == "__main__":
    #record_data(remote_control_cam = True, record_video = False)
    cam_in_world, used = get_camera_poses(output_directory)

    all_imgs = [f for f in os.listdir(output_directory) if f.endswith('.JPG')]
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()
    wrist_in_robot = get_wrists_poses(output_directory)
    wrist_in_robot = np.array(wrist_in_robot)[indices] # [x, y, z, theta_x, theta_y, theta_z]

    coordinate(cam_in_world, wrist_in_robot)
    pass