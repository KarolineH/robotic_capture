#!/usr/bin/env python3.8

import os
import time
import json
import numpy as np
import cv2
import yaml
import pathlib
import datetime

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
import kinova_arm_if.helpers.conversion as conv
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io

''' 
Routine for calibrating the eye-in-hand coordination, that is the transform between robot arm (wrist) and the mounted camera's principal point.
This should only need to be performed whenever the camera is moved or the dimensions of the camera, mount, or lens change.

This routine will move the robot through number of target states and record both the wrist poses and a full-resolution image in each state.
You can optionally record a 1080p HDMI video feed from the camera instead of the full still images. 
It is not advised to do both at the same time but you can run the routine twice if you need both types of data.

Place an AprilTag calibration pattern down in the robot's workspace, and make sure the camera can see it from all angles.
Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
If recording the HDMI feed, the camera should still be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''

def record_data(capture_params=[32,'AUTO','AUTO',True], use_hdmi_stream = False, hdmi_device = '/dev/video2', output_directory='/home/kh790/data/calibration_imgs/eye_in_hand', sleep_time=2):
    '''
    Move the robot through a series of states, capture full-res still images OR 1080p HDMI video along with robot wrist poses at each state.
    Input parameters:
    capture_params: list of camera capture parameters, these are by default set to minimise Bokeh effects (small aperture) and to facilitate better focus on the calibration pattern.
    use_hdmi_stream: if True, the HDMI video stream is recorded, If False, a burst of full-resolution still images is captured.
    hdmi_device: the device name of the HDMI capture device, by default this is set to /dev/video2 (not needed if use_hdmi_stream is False)
    sleep_time: the time to wait after each robot motion before capturing an image, to allow the robot to settle and reduce motion blur.
    output_directory: the directory where the images or video will be saved. A sub-directory will be created at the location for each run.
    Returns the directory where the images or video are saved.
    '''

    # Prepare the image/video output directory
    if not os.path.exists(output_directory):
        print(f"Image output directory {output_directory} does not exist. Please specify an existing location and try again.")
        return
    else:
        # create a new sub-directory at the target location so that images from each run are kept separate
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        im_dir = os.path.join(output_directory, stamp)
        os.mkdir(im_dir)

    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is enabled, you can instead add a focus operation before each capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)
    if use_hdmi_stream:
        rec = Recorder(hdmi_device) # instantiate the HDMI video recorder object

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)


        # TODO: Improve the motion sequence, maybe add more states
        sequence, action_list = data_io.read_action_from_file(actions_dir + "/hand_eye_sequence.json")

        # Start recording
        if use_hdmi_stream:
            rec.start_recording(os.path.join(im_dir,'hand_eye_calibration.mp4'))

        poses = []
        for i,state in enumerate(action_list):
            IF.execute_action(state)
            time.sleep(sleep_time) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
            wrist_pose = IF.get_pose()
            poses.append(wrist_pose)
            if not use_hdmi_stream:
                path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

        # Finally, stop the recording/capture
        if use_hdmi_stream:           
            rec.stop_recording() # stop the recording

    pose_data = [pose.tolist() for pose in poses]
    json.dump(pose_data, open(os.path.join(im_dir, 'hand_eye_wrist_poses.json'), 'w'))
    return im_dir

def get_camera_poses(im_dir, calibrate_camera=False, cam_calib_file=None):
    '''
    Retrieve camera poses from the images taken during the hand-eye calibration routine.
    This is done via AprilTag detection and OpenCV camera calibration.
    Optionally also calibrate the intrinsic camera parameters using the same image collection if not loading intrinsics from a file.
    If calibrate_camera is set to True, the intrinsic camera parameters are also saved to file.
    Returns the camera poses in the world frame and the names of the images that were used for calibration. Some images might not have been suitable because AprilTags were not detected.
    '''

    cc = CamCalibration('mounted_camera', im_dir)
    if not calibrate_camera and os.path.exists(cam_calib_file):
        # load the intrinsic camera parameters from a file
        cam_name, frame_size, matrix, distortion = calibration_io.load_from_yaml(cam_calib_file)
        # then evaluate the images and get the extrinsics, using the loaded intrinsics
        __, __, __, cam_in_world,used = cc.april_tag_calibration(matrix, distortion, lower_requirements=True)
    else:
        # alternatively calibrate both intrinsics and extrinsics from the images
        frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True)
        if calibrate_camera:
            # save the intrinsic camera parameters to a file
            if cam_calib_file is None:
                cam_calib_file = os.path.join(im_dir, 'camera_info.yaml')
            calibration_io.save_to_yaml(cam_calib_file, cc.name, frame_size, matrix, distortion)
    return cam_in_world, used

def get_wrists_poses(directory):
    '''
    Retrieve the wrist poses recording during the hand-eye calibration routine from a file.
    '''
    with open(os.path.join(directory, 'hand_eye_wrist_poses.json'), 'r') as f:
        wrist_poses = json.load(f)
    return wrist_poses

def coordinate(cam_coords, wrist_coords):
    '''
    This function performs two methods for eye-in-hand coordination, optimising the transform between robot wrist and camera.
    Returns the following transforms:
    R_cam2wrist, t_cam2wrist: transforms a point given in the camera to the wrist frame
    R_base2world, t_base2world: transforms a point given in the robot (base) frame to the "world" frame, which is the AprilTag pattern origin
    R_wrist2cam, t_wrist2cam: transforms a point given in the wrist frame to the camera frame, should be the inverse of R_cam2wrist, t_cam2wrist transform
    '''

    # Wrist poses are given as [x, y, z, theta_x, theta_y, theta_z]
    # We convert them to 4x4 homogeneous transformation matrices
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

        # build 4x4 homogeneous transformation matrix
        M = np.zeros((4,4))
        M[:3,:3] = rmat
        M[:3,3] = tvec
        M[3,3] = 1

        # also get the inverse of this transform, which is needed for the second method
        Mi = np.linalg.inv(M) # robot base in wrist frame
        base_R.append(Mi[:3,:3])
        base_t.append(Mi[:3,3])
    
    wrist_R = np.array(wrist_R)
    wrist_t = np.array(wrist_t)
    base_R = np.array(base_R) # inverse of the above
    base_t = np.array(base_t) # inverse of the above

    # Camera poses are given as 4x4 homogeneous transformation matrices
    # they are camera locations, given in the pattern frame
    # also need to be inverted
    pattern_in_cam = []
    for mat in cam_coords:
        inv_mat = np.linalg.inv(mat)
        pattern_in_cam.append(inv_mat)
    pattern_in_cam = np.array(pattern_in_cam) # pattern given in camera frame
    
    cam_R = pattern_in_cam[:,:3,:3]
    cam_t = pattern_in_cam[:,:3,3]

    # Method 1: Only returns camera<>wrist transform
    R_cam2wrist, t_cam2wrist = cv2.calibrateHandEye(wrist_R, wrist_t, cam_R, cam_t)# method=cv2.CALIB_HAND_EYE_PARK)
    # there are also different method implementations available, see (HandEyeCalibrationMethod)
    # output is the camera pose relative to the wrist pose, so the transformation from the wrist to the camera

    # Method 2: Returns both camera<>wrist and base<>pattern transform to anker both in a common world frame
    R_base2world, t_base2world, R_wrist2cam, t_wrist2cam = cv2.calibrateRobotWorldHandEye(cam_R, cam_t, base_R, base_t)

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


    # First run the recording routine
    data_dir = record_data(use_hdmi_stream = False, output_directory='/home/kh790/data/calibration_imgs/eye_in_hand', sleep_time=2)

    # if the camera intrinsics are already calibrated, use those parameters
    config_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config'
    cam_calib_file = os.path.join(config_dir, sorted(os.listdir(config_dir))[0])
    cam_in_world, used = get_camera_poses(data_dir, calibrate_camera=False, cam_calib_file=cam_calib_file)

    # Find the wrist poses that correspond to the images which were successfully used for calibration
    all_imgs = [f for f in os.listdir(data_dir) if f.endswith('.JPG')]
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()
    wrist_in_robot = get_wrists_poses(data_dir)
    wrist_in_robot = np.array(wrist_in_robot)[indices] # [x, y, z, theta_x, theta_y, theta_z]

    # finally, perform calibration
    R_cam2wrist, t_cam2wrist, R_base2world, t_base2world, R_wrist2cam, t_wrist2cam = coordinate(cam_in_world, wrist_in_robot)
    # and save the relevant robot chain transform to file
    save_coordination(R_cam2wrist, t_cam2wrist)