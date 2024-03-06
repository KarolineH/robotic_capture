#!/usr/bin/env python3.8

import os
import time
import json
import numpy as np
import cv2
import yaml
import pathlib

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
    if remote_control_cam:
        cam = EOS() # instantiate the camera interface object
        cam.set_capture_parameters(aperture='AUTO', iso='AUTO', shutterspeed='AUTO', c_AF=True) # change capture settings if needed
        # for i in range(10):
        #     cam.manual_focus(2)
    if record_video:
        rec = Recorder(camera) # instantiate the HDMI video recorder object

    # Create connection to the robot and get the router
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        # get robot Python interface object
        # and deliberately reset the tool transform so we record the wrist positions, not tool positions 
        IF = Kinova3(router, transform=[0,0,0,0,0,0])
        success = True

        # double check that the robot is in its safe resting position
        rest_action = data_io.read_action_from_file("/home/kh790/ws/robotic_capture/kinova_arm_if/data/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        example_sequence, action_list = data_io.read_action_from_file("/home/kh790/ws/robotic_capture/kinova_arm_if/data/hand_eye_sequence.json")
        poses = []

        if record_video:
            rec.start_recording(os.path.join(output_directory, 'video' ,'hand_eye_calibration.mp4')) # start recording the robot's movement

        for i,state in enumerate(action_list):
            IF.execute_action(state)
            if i > 1 and i < len(action_list)-1:
                time.sleep(2) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
                wrist_pose = IF.get_pose()
                poses.append(wrist_pose)
                if remote_control_cam:
                    #cam.trigger_AF() # trigger the camera to focus
                    path, msg = cam.capture_image(download=True, target_path=output_directory) # capture an image and download it to the specified directory
                    print(msg)
        if record_video:           
            rec.stop_recording()

        pose_data = [pose.tolist() for pose in poses]
        json.dump(pose_data, open(os.path.join(output_directory, 'hand_eye_wrist_poses.json'), 'w'))


def get_camera_poses(output_directory, calibrate_camera=True, cam_calib_file='calibration.yaml'):
    '''
    Retrieve camera poses from the images taken during the hand-eye calibration routine.
    This is done via AprilTag detection and camera calibration.
    Optionally also calibrate the intrinsic camera parameters using the same image collection, or load intrinsics from a file.
    '''

    cc = CamCalibration('mounted_camera', output_directory)
    if not calibrate_camera and os.path.exists(cam_calib_file):
        # load the intrinsic camera parameters from a file
        cam_name, frame_size, matrix, distortion = calibration_io.load_from_yaml(cam_calib_file)
        # then evaluate the images and get the extrinsics, using the loaded intrinsics
        __, __, __, cam_in_world,used = cc.april_tag_calibration(matrix, distortion, lower_requirements=True)
    else:
        # alternatively calibrate both intrinsics and extrinsics from the images
        frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True)
        # save the intrinsic camera parameters to a file
        calibration_io.save_to_yaml(cam_calib_file, cc.name, frame_size, matrix, distortion)
    return cam_in_world, used

def get_wrists_poses(output_directory):
    with open(os.path.join(output_directory, 'hand_eye_wrist_poses.json'), 'r') as f:
        wrist_poses = json.load(f)
    return wrist_poses

def coordinate(cam_coords, wrist_coords):
    # TODO: Wrist poses are given as [x, y, z, theta_x, theta_y, theta_z]

    wrist_R = []
    wrist_t = []
    base_R = []
    base_t = []

    for pose in wrist_coords:
        pose = np.array(pose)
        tvec = pose[:3]
        euler_angles = pose[3:]
        rmat = conv.euler_to_mat(*euler_angles)
        wrist_R.append(rmat)
        wrist_t.append(tvec)

        M = np.zeros((4,4))
        M[:3,:3] = rmat
        M[:3,3] = tvec
        M[3,3] = 1
        Mi = np.linalg.inv(M) # robot base in wrist frame
        base_R.append(Mi[:3,:3])
        base_t.append(Mi[:3,3])
    
    wrist_R = np.array(wrist_R)
    wrist_t = np.array(wrist_t)
    base_R = np.array(base_R) # inverse of the above
    base_t = np.array(base_t) # inverse of the above

    # Camera poses are given as 4x4 homogeneous transformation matrices
    # I think these need to be inverted though, it calls for the inverse transformation
    pattern_in_cam = []
    for mat in cam_coords:
        inv_mat = np.linalg.inv(mat)
        pattern_in_cam.append(inv_mat)
    pattern_in_cam = np.array(pattern_in_cam)
    
    cam_R = pattern_in_cam[:,:3,:3]
    cam_t = pattern_in_cam[:,:3,3]

    R_cam2wrist, t_cam2wrist = cv2.calibrateHandEye(wrist_R, wrist_t, cam_R, cam_t)# method=cv2.CALIB_HAND_EYE_PARK)
    # there are also different methods available, see (HandEyeCalibrationMethod)
    # output is the camera pose relative to the wrist pose, so the transformation from the wrist to the camera

    R_base2world, t_base2world, R_wrist2cam, t_wrist2cam = cv2.calibrateRobotWorldHandEye(cam_R, cam_t, base_R, base_t)#, method=cv2.CALIB_HAND_EYE_PARK)
    # maybe use cv2.calibrateRobotWorldHandEye() instead, it spits out the pattern location as well.

    return R_cam2wrist, t_cam2wrist, R_base2world, t_base2world, R_wrist2cam, t_wrist2cam

def save_coordination(R,t):
     # save the transformation to a yaml file
    config_path = str(pathlib.Path(__file__).parent.resolve()) + '/config/frame_transform.yaml'
    
    # convert to Euler angles
    euler = conv.mat_to_euler(R)
    euler_rad = np.deg2rad(euler)

    data = {'frame_id': '',
    'x_translation': float(t[0][0]),
    'y_translation': float(t[1][0]),
    'z_translation': float(t[2][0]),
    'theta_x': float(euler_rad[0]),
    'theta_y': float(euler_rad[1]),
    'theta_z': float(euler_rad[2])
    }
    yaml.dump(data, open(config_path, 'w'), default_flow_style=False)
    return

if __name__ == "__main__":
    #record_data(remote_control_cam = True, record_video = False)
    cam_in_world, used = get_camera_poses(output_directory)

    all_imgs = [f for f in os.listdir(output_directory) if f.endswith('.JPG')]
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()
    wrist_in_robot = get_wrists_poses(output_directory)
    wrist_in_robot = np.array(wrist_in_robot)[indices] # [x, y, z, theta_x, theta_y, theta_z]

    R_cam2wrist, t_cam2wrist, R_base2world, t_base2world, R_wrist2cam, t_wrist2cam = coordinate(cam_in_world, wrist_in_robot)
    save_coordination(R_cam2wrist, t_cam2wrist)