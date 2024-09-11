#!/usr/bin/env python3.8

import os
import datetime
import numpy as np
import time

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.kbhit as kbhit
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv


def main(out_dir='/home/kh790/data/scans', capture_params=[32,'AUTO','AUTO',False], wait_time=None, use_wrist_frame=False, goal_states=[]):

    ''' 
    Move through a sequence of goal states (np array of nx6 joint angles, given in degrees).
    At each goal, capture a still image, record the measured camera pose and the measured joint angles.
    Save the images and the recorded data to files in the specified output directory.
    Wait_time is the time to wait at each goal state before capturing an image, can be set to None to wait for a key press instead.

    WARNING: Make sure the robot is in a suitable starting position BEFORE running this script.
    '''

    # create new directory for the current capture session
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = os.path.join(out_dir, stamp)
    os.mkdir(im_dir)
    # Instantiate the two log files, recording the measured states and measured camera poses
    states_file = os.path.join(im_dir, 'visited_states.txt')
    poses_file = os.path.join(im_dir, 'raw_poses.txt')
    im_names_file = os.path.join(im_dir, 'im_names.txt') 

    # format the goal states as Kinova robot actions
    actions = [data_io.create_angular_action(np.asarray(goal)) for goal in goal_states]

    # instantiate the camera interface object (default: small aperture to reduce Bokeh effects)
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, use_wrist_frame=use_wrist_frame)
        print(IF.camera_frame_transform) # sanity check, is this the correct frame?

        success = True

        # Start manoeuvring the robot to the desired position
        for i,action in enumerate(actions):
            print(f"Moving to goal state {i+1}/{len(actions)}")
            success &= IF.execute_action(action)
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
                np.savetxt(f,state.reshape(1,-1), delimiter=',')
            with open(poses_file, "ab") as f:
                np.savetxt(f,pose.reshape(1,-1), delimiter=',')
            with open(im_names_file, "a") as f:
                f.write(f"{path}\n")

        print("Capturing finished")

    poses = np.loadtxt(poses_file, delimiter=',')
    pose_data = [pose.tolist() for pose in poses]
    f = open(im_names_file, 'r')
    im_files = f.readlines()
    file_names = [os.path.basename(f) for f in im_files]
    poses_to_txt(pose_data, file_names, os.path.join(im_dir, 'cam_poses.txt'))
    return

def poses_to_txt(pose_data, file_names, path):
    # First, save the raw poses to a separate file for future reference and debugging.
    # raw_poses = np.asarray(pose_data)
    # raw_file = '/'.join(path.split('/')[:-1]) + '/raw_poses.txt'
    # np.savetxt(raw_file, raw_poses, delimiter=',', comments='x,y,z,theta_x,theta_y,theta_z')

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
            # COLMAP expects image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, file name
            f.write(f"{i+1} {quaternions[i][0]} {quaternions[i][1]} {quaternions[i][2]} {quaternions[i][3]} {ts[i][0]} {ts[i][1]} {ts[i][2]} 1 {file_names[i]}\n")
            f.write("\n") # add a newline between each pose, COLMAP expects this

if __name__ == "__main__":

    common_paths = {'scan': '/home/kh790/data/scans', 'coord': '/home/kh790/data/calibration_imgs/eye_hand', 'intrinsics': '/home/kh790/data/calibration_imgs/intrinsics'} 

    #select a target path for the recording
    output_directory = common_paths['scan']

    # set desired parameters
    capture_params=[22,'AUTO','AUTO',False] # aperture, shutter speed, ISO, continuous autofocus
    wait_time = 5 #seconds, wait time can be 0 if you want to keep moving, it can also be none to wait for a key press instead of a timer
    use_wrist_frame = True # optionally record the pose of the robot end-effector/wrist frame instead of the camera pose, so as to not use a previously calibrated camera frame transform
    
    #states = np.array([[0,0,0,0,0,0],[30,0,0,0,0,0],[15,0,0,0,0,0]]) # np array of joint states, nx6
    states = np.loadtxt('/home/kh790/data/paths/intrinsics_calib.txt', delimiter=',') # a pre-recorded path of joint states loaded from file

    main(output_directory, capture_params, wait_time, use_wrist_frame, states)