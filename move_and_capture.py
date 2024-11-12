#!/usr/bin/env python3.8

import os
import datetime
import numpy as np
import time
import pathlib

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.kbhit as kbhit
import kinova_arm_if.helpers.data_io as data_io
import kinova_arm_if.helpers.wrist_to_cam_tf as wrist_to_cam
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv
from calibration.helpers import calibration_io


def capture(out_dir='/home/kh790/data/scans', capture_params=[32,'AUTO','AUTO',False], focus_dist=10, wait_time=None, goal_states=[]):

    ''' 
    Move through a sequence of goal states (np array of nx6 joint angles, given in degrees).
    At each goal, capture a still image, record the measured wrist pose and the measured joint angles.
    Save the images and the recorded data to files in the specified output directory.
    Wait_time is the time to wait at each goal state before capturing an image, can be set to None to wait for a key press instead.

    WARNING: Make sure the robot is in a suitable starting position BEFORE running this script.
    '''

    # create new directory for the current capture session
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = os.path.join(out_dir, stamp)
    os.mkdir(im_dir)
    # Instantiate the log files, recording the measured states and measured poses
    states_file = os.path.join(im_dir, 'visited_states.txt')
    poses_file = os.path.join(im_dir, 'raw_poses.txt') # raw poses are the robot wrist (=end-effector) poses, not the camera poses
    cam_transforms_file = os.path.join(im_dir, 'cam_tfs.txt') # camera poses w.r.t. the robot base frame, saves as homogeneous transformation matrices
    im_names_file = os.path.join(im_dir, 'im_names.txt')
    with open(poses_file, "w") as f:
        f.write(f'# Poses are given as the robot wrist frame w.r.t. the robot base frame\n')

    # format the goal states as Kinova robot actions
    actions = [data_io.create_angular_action(np.asarray(goal)) for goal in goal_states]

    # instantiate the camera interface object (default: small aperture to reduce Bokeh effects)
    cam = EOS()
    cam.set_capture_parameters(*capture_params)
    cam.fixed_focus(focus_dist) # set the camera to a specified focus distance

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, use_wrist_frame=True) # always record the wrist poses
        print(IF.camera_frame_transform) # sanity check, is this the correct frame?

        success = True 

        # Start manoeuvring the robot through the desired positions
        for i,action in enumerate(actions):
            print(f"Moving to goal state {i+1}/{len(actions)}")
            success &= IF.execute_action(action)
            if i==0:
                success &= IF.execute_action(action) # the first action is sometimes executed without locking the thread, make sure it has been executed before moving on
            if wait_time is not None:
                time.sleep(wait_time)
            else:
                checking = True
                print("Goal state reached, press any key to capture and continue")
                while checking:
                    if kbhit.KBHit().kbhit():
                        checking = False

            pose = IF.get_pose()
            state = IF.get_joint_angles()
            path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

            with open(states_file, "ab") as f:
                np.savetxt(f,state.reshape(1,-1), delimiter=',') # states are saved to file line by line
            with open(poses_file, "ab") as f:
                np.savetxt(f,pose.reshape(1,-1), delimiter=',') # raw poses are saved to file line by line
            with open(im_names_file, "a") as f:
                f.write(f"{path}\n") # image names are saved to file line by line

        print("Capturing finished")

    with open(im_names_file, 'r') as f:
        im_files = f.readlines()
    im_file_names = [os.path.basename(f).strip() for f in im_files]

    cam_tfs = wrist_to_cam.wrist_to_cam_poses(poses_file) # convert the raw wrist poses to camera poses using the latest eye-in-hand calibration results, in robot base frame
    reshaped_cam_tfs = cam_tfs.reshape(cam_tfs.shape[0],-1)
    np.savetxt(cam_transforms_file, reshaped_cam_tfs, delimiter=',') # save the camera poses in the robot base frame
    poses_to_colmap_format(cam_tfs, im_file_names, os.path.join(im_dir, 'images.txt')) # save the camera poses in the format required by COLMAP
    create_cameras_file(os.path.join(im_dir, 'cameras.txt')) # save the camera intrinsics in the format required by COLMAP
    # TODO also save poses in the NeRF format
    from calibration.helpers import plotting
    plotting.plot_transforms(cam_tfs)
    return

def poses_to_colmap_format(tfs, file_names, path):
    # 1. COLMAP expects the transformation from world to camera frame, not from camera to world frame. 
    # Need to invert the transformation. We will do this in homogeneous transformation matrix form.
    inv_transforms = np.asarray([conv.invert_transform(mat) for mat in tfs]) # invert the transforms
    Rs = inv_transforms[:, :3, :3]
    ts = inv_transforms[:, :3, 3]

    # 2. COLMAP expects the camera poses in quaternion format, while the robot gives them in euler angles. Need to convert the euler angles to quaternions.
    quaternions = conv.mat_to_quat(Rs)
    # reorder the quaternion to scalar-first format
    quaternions = np.concatenate((quaternions[:,3].reshape(-1,1), quaternions[:,:3]), axis=1)

    # 3. Formatting and saving to txt file
    with open(path, 'w') as f:
        f.write("# id, QW, QX, QY, QZ, TX, TY, TZ, camera_id\n")
        for i in range(len(tfs)):
            # COLMAP expects image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, file name
            f.write(f"{i+1} {quaternions[i][0]} {quaternions[i][1]} {quaternions[i][2]} {quaternions[i][3]} {ts[i][0]} {ts[i][1]} {ts[i][2]} 1 {file_names[i]}\n")
    return

def create_cameras_file(path):
    calib_file=calibration_io.fetch_recent_intrinsics_path('EOS01')
    cam_id, frame_size, matrix, distortion, cam_model = calibration_io.load_intrinsics_from_yaml(calib_file)

    if cam_model == 'SIMPLE_PINHOLE':
        param_string = f"{np.around(matrix[0,0],decimals=6)} {np.around(matrix[0,2],decimals=6)} {np.around(matrix[1,2],decimals=6)}" # f, cx, cy
    elif cam_model == 'OPENCV':
        param_string = f"{np.around(matrix[0,0],decimals=6)} {np.around(matrix[1,1],decimals=6)} {np.around(matrix[0,2],decimals=6)} {np.around(matrix[1,2],decimals=6)} {np.around(distortion[0],decimals=6)} {np.around(distortion[1],decimals=6)} {np.around(distortion[2],decimals=6)} {np.around(distortion[3],decimals=6)}" # fx, fy, cx, cy, k1, k2, p1, p2
    elif cam_model == 'FULL_OPENCV':
        param_string = f"{np.around(matrix[0,0],decimals=6)} {np.around(matrix[1,1],decimals=6)} {np.around(matrix[0,2],decimals=6)} {np.around(matrix[1,2],decimals=6)} {np.around(distortion[0],decimals=6)} {np.around(distortion[1],decimals=6)} {np.around(distortion[2],decimals=6)} {np.around(distortion[3],decimals=6)} {np.around(distortion[4],decimals=6)}" # fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6

    with open(path, 'w') as f:
        f.write("# Camera list with one line of data per camera:\n")
        f.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
        f.write("# Number of cameras: 1\n")
        f.write(f"1 {cam_model} {frame_size[0]} {frame_size[1]} {param_string}\n")
    print("camera details saved to file in COLMAP format")
    return

def main(directory):

    # Set up the capture parameters:
    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config' # default location is the config directory of this package
    __, min_aperture, focus_dist = calibration_io.load_lens_config(calibr_dir + '/lens_config.yaml', lens_id=0)
    capture_params=[min_aperture,'AUTO','AUTO',False] # aperture, shutter speed, ISO, continuous autofocus
    wait_time = 5 #seconds, wait time can be 0 if you want to keep moving, it can also be none to wait for a key press instead of a timer

    # Specify the series of joint states for the robot to visit and capture images at:
    #states = np.array([[0,0,0,0,0,0],[30,0,0,0,0,0],[15,0,0,0,0,0]]) # np array of joint states, nx6
    #states = np.loadtxt('/home/kh790/data/paths/scan_path.txt', delimiter=',') # a pre-recorded path of joint states loaded from file
    states = np.loadtxt('/home/kh790/data/paths/intrinsics_calib.txt', delimiter=',')

    capture(directory, capture_params, focus_dist, wait_time, states)


if __name__ == "__main__":
    # Define your preffered target directories for the collected data
    directories = {
        'scan': ('/home/kh790/data/scans'), # for most scans
        'coord': ('/home/kh790/data/calibration_imgs/eye_hand'), # for hand-eye calibration
        'intrinsics': ('/home/kh790/data/calibration_imgs/intrinsics') # for camera (intrinsics) calibration
        }
    
    # Pick a target directory
    target_dir = directories['scan']
    main(target_dir)