#!/usr/bin/env python3.8

import os
import datetime
import numpy as np
import pathlib 

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.kbhit as kbhit
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv
from calibration.helpers import calibration_io

def main(out_dir='/home/kh790/data/scans', capture_params=[22,'AUTO','AUTO',False], focus_dist=10, use_wrist_frame=False):

    ''' 
    Capture a series of still images, along with the measured camera/wrist pose and the measured joint angles.
    Save the images and the recorded data to files in the specified output directory.
    The robot movements should be controlled manually, this script only captures images.
    '''
        
    # create new directory for the current capture session
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = os.path.join(out_dir, stamp)
    os.mkdir(im_dir)
    states_file = os.path.join(im_dir, 'states.txt')
    poses_file = os.path.join(im_dir, 'raw_poses.txt')

    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is enabled, you can instead add a focus operation before each capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)
    cam.fixed_focus(focus_dist)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, use_wrist_frame=use_wrist_frame)
        print(IF.camera_frame_transform)
        success = True
        running=True

        # Start manoeuvring the robot to the desired position
        files = []
        poses = []
        states = []

        print("Press 'c' to capture an image, 'q' to quit")

        while running:
            keyboard = kbhit.KBHit()
            if keyboard.kbhit():
                # if a button was pressed
                q = keyboard.getch() # get the character

                if q == 'c': # c for capture    
                    cam_pose = IF.get_pose() # [x, y, z, theta_x, theta_y, theta_z]
                    joint_state = IF.get_joint_angles()
                    path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory
                    files.append(path)
                    poses.append(cam_pose)
                    states.append(joint_state)
                    print(f"Captured image: {path}")

                    with open(states_file, "ab") as f:
                        np.savetxt(f,joint_state.reshape(1,-1),  delimiter=',')
                    with open(poses_file, "ab") as f:
                        np.savetxt(f,cam_pose.reshape(1,-1), delimiter=',')
                        
                if q == 'q': # q for quit
                    running = False

        print("Capturing finished")

    pose_data = [pose.tolist() for pose in poses]
    state_data = [state.tolist() for state in states]
    file_names = [os.path.basename(f) for f in files]
    np.savetxt(os.path.join(im_dir, 'states.txt'), np.asarray(state_data), delimiter=',')
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
    output_directory = '/home/kh790/data/paths'

    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config' # default location is the config directory of this package
    __, min_aperture, focus_dist = calibration_io.load_lens_config(calibr_dir + '/lens_config.yaml', lens_id=0)


    main(output_directory, focus_dist=focus_dist, use_wrist_frame=True)