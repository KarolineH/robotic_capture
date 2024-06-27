#!/usr/bin/env python3.8

import os
import numpy as np
import cv2
import yaml
import pathlib
import kinova_arm_if.helpers.conversion as conv
from calibration.calibrate import CamCalibration
from calibration.helpers import calibration_io as calibration_io

''' 
Routine for calibrating the eye-in-hand coordination, that is the transform between robot arm (wrist) and the mounted camera's principal point.
This should only need to be performed whenever the camera is moved or the dimensions of the camera, mount, or lens change.

Specify a directory with the required data, you need a set of images of an AprilTag pattern and their corresponding wrist poses. Note that these need to be robot wrist (or end-effector) poses, not camera poses.
Images should be .jpg files and wrist poses should be in a text file named 'poses.txt' in the same directory.
Record this using the move_and_capture.py script, or semi-automatically using the capture_only.py script.
'''

def load_wrist_poses(directory):
    poses = np.loadtxt(os.path.join(directory, 'raw_poses.txt'), delimiter=',') # [n,6]
    return poses

def get_camera_poses(im_dir, cam_id='EOS01', calibrate_intrinsics=False):
    '''
    Retrieve camera poses from the images taken during the hand-eye calibration routine.
    This is done via AprilTag detection and OpenCV camera calibration.
    Optionally also calibrate the intrinsic camera parameters using the same image collection if not loading intrinsics from a file.
    If calibrate_intrinsics is set to True, the intrinsic camera parameters are also saved to file.
    Returns the camera poses in the world frame and the names of the images that were used for calibration. Some images might not have been suitable because AprilTags were not detected.
    '''

    cc = CamCalibration(cam_id, im_dir)

    if not calibrate_intrinsics:
        # find the most recent camera calibration file and load the intrinsic camera parameters
        intrinsics_file = calibration_io.fetch_recent_intrinsics_path(cam_id)
        if intrinsics_file is not None:
            print(f"Loading camera intrinsics from {intrinsics_file}")
            cam_name, frame_size, matrix, distortion, model = calibration_io.load_intrinsics_from_yaml(intrinsics_file)
            # then evaluate the images and get the extrinsics, using the loaded intrinsics
            print("Calibrating extrinsics based on AprilTag detection.")
            __, __, __, cam_in_world,used = cc.april_tag_calibration(matrix, distortion, lower_requirements=True, cam_model=model)
            return cam_in_world, used
        
    # alternatively calibrate both intrinsics and extrinsics from the image set
    distortion_model = 'OPENCV' # the camera model used for calibration
    frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True, cam_model=distortion_model)
    stamp = im_dir.split('/')[-1]
    cam_calib_file = str(pathlib.Path(__file__).parent.resolve()) + f'/config/camera_info_{cam_id}_{stamp}.yaml'
    calibration_io.save_intrinsics_to_yaml(cam_calib_file, cam_id, distortion_model, frame_size, matrix, distortion)
    return cam_in_world, used

def coordinate(cam_coords, wrist_coords):
    '''
    This function performs two methods for eye-in-hand coordination, optimising the transform between robot wrist and camera.
    Returns the following transforms:
    cam_in_wrist_tf, 
    pattern_in_base_tf
    '''

    # Wrist poses are given as [x, y, z, theta_x, theta_y, theta_z] in meters and degrees, w.r.t the robot base
    # We convert them to 4x4 homogeneous transformation matrices
    print("Calculating the hand-eye coordination...")

    wrist_in_base = conv.robot_poses_as_htms(wrist_coords) # robot wrist poses w.r.t. base frame [n,4,4]
    base_in_wrist = np.asarray([np.linalg.inv(mat) for mat in wrist_in_base]) # robot base poses w.r.t. wrist frame [n,4,4]

    base_R = base_in_wrist[:,:3,:3]
    base_t = base_in_wrist[:,:3,3]

    # Camera poses are given as 4x4 homogeneous transformation matrices, w.r.t. the calibration pattern origin frame ('world')
    # = camera locations, given in the pattern frame
    # also need to be inverted
    pattern_in_cam = np.asarray([np.linalg.inv(mat) for mat in cam_coords]) # pattern in camera frame [n,4,4]
    cam_R = pattern_in_cam[:,:3,:3]
    cam_t = pattern_in_cam[:,:3,3]

    # Method 1: Only returns camera<>wrist transform
    # R_cam2wrist, t_cam2wrist = cv2.calibrateHandEye(wrist_in_base[:,:3,:3], wrist_in_base[:,:3,3], pattern_in_cam[:,:3,:3], pattern_in_cam[:,:3,3])# method=cv2.CALIB_HAND_EYE_PARK)
    # there are also different method implementations available, see (HandEyeCalibrationMethod)
    # output is the camera pose relative to the wrist pose, so the transformation from the wrist to the camera

    # Method 2: Returns both camera<>wrist and base<>pattern transform to anker both in a common world frame
    # From initial tests, this method seems to yield more accurate results
    R_base_in_pattern, t_base_in_pattern, R_wrist_in_cam, t_wrist_in_cam = cv2.calibrateRobotWorldHandEye(cam_R, cam_t, base_R, base_t, method=cv2.CALIB_HAND_EYE_TSAI)
    # R_base2world, t_base2world: transforms a point given in the robot (base) frame to the "world" frame, which is the AprilTag pattern origin. The is the same as the robot base pose expressed in the pattern origin frame
    # R_wrist_in_cam, t_wrist_in_cam: transforms a point given in the wrist frame to the camera frame. This is the same as the wrist pose expressed in the camera frame.

    base_in_pattern_tf = np.zeros((4,4))
    base_in_pattern_tf[:3,:3] = R_base_in_pattern
    base_in_pattern_tf[:3,3] = t_base_in_pattern.flatten()
    base_in_pattern_tf[3,3] = 1

    pattern_in_base_tf = np.linalg.inv(base_in_pattern_tf) # the pose of the calibration pattern expressed in the robot base frame

    wrist_in_cam_tf = np.zeros((4,4))
    wrist_in_cam_tf[:3,:3] = R_wrist_in_cam
    wrist_in_cam_tf[:3,3] = t_wrist_in_cam.flatten()
    wrist_in_cam_tf[3,3] = 1

    cam_in_wrist_tf = np.linalg.inv(wrist_in_cam_tf) # the transform from the camera to the wrist frame, or the pose of the camera expressed in the wrist frame

    # R_cam_in_wrist = cam_in_wrist_tf[:3,:3] # the rotation from the camera frame to the wrist frame, or the rotation of the camera expressed in the wrist frame
    # t_cam_in_wrist = cam_in_wrist_tf[:3,3] # the translation from the camera frame to the wrist frame, or the translation of the camera expressed in the wrist frame

    return cam_in_wrist_tf, pattern_in_base_tf

def save_coordination(cam_tf,pattern_tf=None,stamp=''):
     # save the camera transformation (camera pose expressed in wrist frame) to a yaml file
    config_path = str(pathlib.Path(__file__).parent.resolve()) + f'/config/dslr_transform_{stamp}.yaml'

    data = {
        'frame_id': 'dslr_frame',
        'parent_frame': 'end_effector_link',
        'transform': cam_tf.tolist()
    }
    yaml.dump(data, open(config_path, 'w'), default_flow_style=False)

    if pattern_tf is not None:
        # If given, also save the world to base (robot base pose expressed in pattern origin frame) transform 
        pattern_path = str(pathlib.Path(__file__).parent.resolve()) + f'/config/pattern_in_base_tf_{stamp}.yaml'
        data = {
        'frame_id': 'calibration_pattern',
        'parent_frame': 'base_link',
        'transform': pattern_tf.tolist()}
        yaml.dump(data, open(pattern_path, 'w'), default_flow_style=False)
    return

def main(data_dir, cam_id = 'EOS01'):
    cam_in_world, used = get_camera_poses(data_dir, cam_id, calibrate_intrinsics=True) # calculate camera poses from images of the calibration pattern
    print(f"{len(used)} images were useable for eye-in-hand calibration")

    # Find the wrist poses that correspond to the images which were successfully used for calibration
    all_imgs = [f for f in os.listdir(data_dir) if f.endswith('.JPG')]
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()

    wrist_in_robot = load_wrist_poses(data_dir)
    wrist_in_robot = wrist_in_robot[indices] # [x, y, z, theta_x, theta_y, theta_z]

    # finally, perform calibration
    cam_in_wrist_tf, pattern_in_base_tf = coordinate(cam_in_world, wrist_in_robot)
    # and save the relevant robot chain transform to file
    save_coordination(cam_in_wrist_tf, pattern_in_base_tf, stamp=data_dir.split('/')[-1])
    print("Hand-eye coordination complete.")

if __name__ == "__main__":
    im_dir = '/home/kh790/data/calibration_imgs/hand_eye_coord/2024-06-27_12-12-37'
    main(im_dir)