#!/usr/bin/env python3.8

import os
import datetime
import numpy as np

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.kbhit as kbhit
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv

def main(out_dir, capture_params=[32,'AUTO','AUTO',False]):
    # create new directory for the current capture session
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = os.path.join(out_dir, stamp)
    os.mkdir(im_dir)
    states_file = os.path.join(im_dir, 'states.txt')
    poses_file = os.path.join(im_dir, 'cam_poses.txt')

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

                    with open(states_file, 'a') as f:
                        f.write(f"{joint_state.tolist()}\n")
                    with open(poses_file, 'a') as f:
                        f.write(f"{cam_pose.tolist()}\n")
                    # TODO: Change this to use numpy savetxt
                    # with open("test.txt", "ab") as f:
                    #   np.savetxt(f, a)
                    #    f.write("\n") # add a newline
                        
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
            # COLMAP expects image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, file name
            f.write(f"{i+1} {quaternions[i][0]} {quaternions[i][1]} {quaternions[i][2]} {quaternions[i][3]} {ts[i][0]} {ts[i][1]} {ts[i][2]} 1 {file_names[i]}\n")
            f.write("\n") # add a newline between each pose, COLMAP expects this

if __name__ == "__main__":
    output_directory = '/home/kh790/data/scans'
    main(output_directory)