#!/usr/bin/env python3.8

import matplotlib.pyplot as plt
import numpy as np
import time
import pathlib
import json
import datetime
import os
import cv2

# import kinova_arm_if.helpers.kortex_util as k_util
# import kinova_arm_if.helpers.data_io as data_io
# from kinova_arm_if.arm_if import Kinova3
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io
from calibration.helpers import transform_util
from eos_camera_if.cam_io import EOS
from kinova_arm_if.helpers import conversion
from calibration.helpers import plotting

def set_vs_measured_states():
    '''
    Move through a sequence of joint states
    Measures the joint angles repeatedly immediately after reaching the state
    Also takes a full-res image of the AprilTag pattern at each state
    Saves the error in measured joint angles compared to the set joint angles and the measurement's time stamps to file
    Also saves the measured cartesian camera pose to file (in the robot's base frame)
    '''
    test_speed_limits = [10,20,30,40]
    num_measurements = 40
    sleep_time = 3
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = f'/home/kh790/data/test_measurements/set_vs_measured_states/{stamp}'
    os.mkdir(im_dir)
    actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
    sequence, action_list = data_io.read_action_from_file(actions_dir + '/calibration_sequence_20.json')
    skip_photos_at = [0,21,22]

    capture_params=[32,'AUTO','AUTO',True]
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe resting position
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        errors_overall = []
        timesteps_overall = []
        poses_overall = []

        for speed in test_speed_limits:
            joint_speeds = [speed] * 6
            IF.set_speed_limit(joint_speeds=joint_speeds, control_mode=4)
            
            errors_by_speed = []
            timesteps_by_speed = []
            poses_by_speed = []

            # Reach a series of joint states
            for i,state in enumerate(action_list):
                target = [joint.value for joint in list(state.reach_joint_angles.joint_angles.joint_angles)]
                IF.execute_action(state)

                errors_by_state = []# [measurements x joints]
                timesteps_by_state = []
                start = time.time()
                for j in range(num_measurements):
                    measured_state = IF.get_joint_angles()
                    diff = np.subtract(measured_state, target)
                    errors_by_state.append(diff)
                    timesteps_by_state.append(time.time() - start)

                errors_by_speed.append(errors_by_state) # [states x measurements x joints]
                timesteps_by_speed.append(timesteps_by_state)

                if i not in skip_photos_at:
                    # unless it is a state where we can skip taking a photo, such as a manouvering or starting pose, take a photo
                    time.sleep(sleep_time) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
                    cam_pose = IF.get_pose()
                    poses_by_speed.append(cam_pose)
                    path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

            errors_overall.append(errors_by_speed)
            timesteps_overall.append(timesteps_by_speed)
            poses_overall.append(poses_by_speed)
        
        errors = np.asarray(errors_overall) # shape [speeds,states,measurements,joints]
        timesteps = np.asarray(timesteps_overall) # shape [speeds,states,measurements,joints]

        with open(os.path.join(im_dir, 'states_errors.npy'), 'wb') as f:
            np.save(f, errors)
        with open(os.path.join(im_dir, 'states_timesteps.npy'), 'wb') as f:
            np.save(f, timesteps)
        poses_unravelled = [pose.tolist() for entry in poses_overall for pose in entry]
        json.dump(poses_unravelled, open(os.path.join(im_dir, 'measured_cam_poses.json'), 'w'))
    return im_dir

def anaylse_joint_errors(im_dir):
    '''
    Plot the errors in the measured joint angles compared to the set joint angles recorded in set_vs_measured_states
    '''

    with open(os.path.join(im_dir, 'states_errors.npy'), 'rb') as f:
        errors = np.load(f)
    with open(os.path.join(im_dir, 'states_timesteps.npy'), 'rb') as f:
        timesteps = np.load(f)

    speeds = [10,20,30,40]

    # Dimensions of the error array
    num_figures, num_lines, num_x_values, num_subplots = errors.shape
    for fig_idx in range(num_figures):
        plt.figure(fig_idx + 1)
        for subplot_idx in range(num_subplots):
            if subplot_idx == 0:
                ax1 = plt.subplot(2, 3, 1)
            else:
                plt.subplot(2, 3, subplot_idx +1, sharey=ax1)
            for line_idx in range(num_lines):
                plt.plot(timesteps[fig_idx, line_idx], errors[fig_idx, line_idx, :, subplot_idx], label=f'Line {line_idx + 1}')
            if subplot_idx == 0:
                plt.legend(['pose 1', 'pose 2', 'pose 3', 'pose 4', 'pose 5', 'pose 6', 'pose 7', 'pose 8', 'pose 9'])
            plt.title(f'Joint nr. {subplot_idx + 1}')
            plt.xlabel('Time [s]')
            plt.ylabel('Error [°]')
            plt.ylim(-0.2, 0.2)
            plt.xlim(0, 0.8)
        plt.suptitle(f'Speed limit {speeds[fig_idx]} [°/s]')

    # Show the plots
    plt.show()
    return

def analyse_pose_errors(im_dir):
    # read file
    with open(os.path.join(im_dir, 'hand_eye_wrist_poses.json'), 'r') as f:
        poses = np.asarray(json.load(f))
    if len([item for item in os.listdir(im_dir) if '.JPG' in item]) != poses.shape[0]:
        raise ValueError('Number of images and number of poses do not match')
    
    # get the camera poses from the images via AprilTag detection and OpenCV calibration, then compare
    cc = CamCalibration('mounted_camera', im_dir)
    # get the most recent camera calibration
    config_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config'
    most_recent_calib = sorted([entry for entry in os.listdir(config_dir) if 'camera_info' in entry])[-1]
    cam_calib_file = os.path.join(config_dir, most_recent_calib) # get the most recent camera calibration file path

    if os.path.exists(cam_calib_file):
        # load the intrinsic camera parameters from a file
        cam_name, frame_size, matrix, distortion = calibration_io.load_from_yaml(cam_calib_file)
        # then evaluate the images and get the extrinsics, using the loaded intrinsics
        __, __, __, cam_in_pattern,used = cc.april_tag_calibration(matrix, distortion, lower_requirements=True)
    else:
        # if not available, calibrate intrinsics and extrinsics from the images
        frame_size,matrix,distortion,cam_in_pattern,used = cc.april_tag_calibration(lower_requirements=True)
    
    # load the transform from the robot base to the world/pattern frame
    most_recent_pattern_transform = sorted([entry for entry in os.listdir(config_dir) if 'pattern2base' in entry])[-1]
    base_transform = os.path.join(config_dir, most_recent_pattern_transform) 
    if os.path.exists(base_transform):
        t = calibration_io.transform_from_yaml(base_transform) # the transform from the robot base to the world frame, or the pose of the robot base expressed in the world/pattern frame
    
    all_imgs = sorted([f for f in os.listdir(im_dir) if f.endswith('.JPG')])
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()
    cam_in_base = poses[indices,:] # [x,y,z,theta_x,theta_y,theta_z]

    # multiply (transform from camera to robot base frame) x (transform from robot base to world frame) to get the (transform from camera to world frame)
    cam_in_base_mat = np.zeros((len(cam_in_base), 4, 4))
    cam_in_base_mat[:,:3,:3] = np.asarray([conversion.euler_to_mat(*pose[3:]) for pose in cam_in_base])
    cam_in_base_mat[:,3,3] = 1
    cam_in_base_mat[:,:3,3] = cam_in_base[:,:3]
    cam_poses_from_robot = t @ cam_in_base_mat # poses of the camera, given in the pattern frame, as derived from the robot's proprioception measurements
    cam_poses_from_images = cam_in_pattern # poses of the camera, given in the pattern frame, as derived from the AprilTag locations in the images

    from calibration.helpers import plotting
    plotting.plot_transforms(cam_poses_from_images)
    plotting.plot_transforms(cam_poses_from_robot)

    diff = np.subtract(cam_poses_from_robot, cam_poses_from_images)

    rot_err = np.asarray([np.linalg.norm(cv2.Rodrigues(cam_poses_from_robot[i,:3,:3].dot(cam_poses_from_images[i,:3,:3].T))[0]) for i in range(diff.shape[0])]) # in radians
    rot_err2 = np.asarray([np.rad2deg(np.arccos((np.trace(cam_poses_from_robot[i,:3,:3] @ cam_poses_from_images[i,:3,:3].T) - 1) / 2)) for i in range(diff.shape[0])]) # in degrees
    # np.testing.assert_almost_equal(np.rad2deg(rot_err), rot_err2) # VERIRY that these are in fact the same
    # methods from https://stackoverflow.com/questions/6522108/error-between-two-rotations
    # and https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices

    # TODO: plot
    fig = plt.figure()
    ax1 = fig.add_subplot(2,2,1)
    x_errors = plt.hist(diff[:,0,3], bins=20)
    plt.title('X errors')
    ax2 = fig.add_subplot(2,2,2)
    y_errors = plt.hist(diff[:,1,3], bins=20)
    plt.title('Y errors')
    ax3 = fig.add_subplot(2,2,3)
    z_errors = plt.hist(diff[:,2,3], bins=20)
    plt.title('Z errors')
    ax4 = fig.add_subplot(2,2,4)
    plt.hist(rot_err2, bins=20)
    plt.title('Rotation errors')
    plt.show()
    return

if __name__ == "__main__":
    #im_dir = set_vs_measured_states()
    im_dir = '/home/karo/ws/data/calibration_images/repeat_test'
    #anaylse_joint_errors(im_dir)
    analyse_pose_errors(im_dir)