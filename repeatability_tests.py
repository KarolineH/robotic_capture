#!/usr/bin/env python3.8

import matplotlib.pyplot as plt
import numpy as np
import time
import pathlib
import json
import datetime
import os
import cv2

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from calibration.calibrate import CamCalibration
from calibration.helpers import calibration_io as calibration_io
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

    This recording routine can take around 30 minutes to complete, depending on the number of states, waiting times, and range of tested speeds.
    Works best when continuous autofocus is enabled and the auto focus mode is set to face tracking.
    '''
    test_speed_limits = [10,20,30,40]
    num_measurements = 40
    sleep_time = 5 # seconds

    # create new directory for the images and measurements
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = f'/home/kh790/data/test_measurements/set_vs_measured_states/{stamp}'
    os.mkdir(im_dir)

    # load the movement sequence
    actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
    sequence, action_list = data_io.read_action_from_file(actions_dir + '/calibration_sequence_44.json')
    skip_capture_at = [0,45] # first and last pose

    # prep the camera
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

                if i not in skip_capture_at:
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
        IF.reset_speed_limit()
    return im_dir

def analyse_joint_errors(im_dir):
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
    # save to png
    for i in range(num_figures):
        plt.figure(i + 1)
        plt.savefig(os.path.join(im_dir, f'joint_errors_speed_{speeds[i]}.png'))
    return

def analyse_pose_errors(im_dir, cam_id):
    '''
    Computation time scales with the number of images, this might take a while.
    '''
    # read file
    with open(os.path.join(im_dir, 'measured_cam_poses.json'), 'r') as f:
        poses = np.asarray(json.load(f))
    if len([item for item in os.listdir(im_dir) if '.JPG' in item]) != poses.shape[0]:
        raise ValueError('Number of images and number of poses do not match')
    
    # get the camera poses from the images via AprilTag detection and OpenCV calibration, then compare
    cc = CamCalibration(cam_id, im_dir)
    # get the most recent camera calibration file (intrinsics), if available
    intrinsics_file = calibration_io.fetch_recent_intrinsics_path(cam_id)
    if intrinsics_file is not None:
        # load the intrinsic camera parameters from a file
        cam_name, frame_size, matrix, distortion, model = calibration_io.load_intrinsics_from_yaml(intrinsics_file)
        # then evaluate the images and get the extrinsics, using the loaded intrinsics
        __, __, __, cam_in_pattern,used = cc.april_tag_calibration(matrix, distortion, lower_requirements=True, cam_model=model)
    else:
        # if not available, calibrate intrinsics and extrinsics from the images
        distortion_model = 'OPENCV' # the camera model used for calibration
        frame_size,matrix,distortion,cam_in_pattern,used = cc.april_tag_calibration(lower_requirements=True, cam_model=distortion_model)
    
    # load the transform from the robot base to the world/pattern frame
    base_transform_file = calibration_io.fetch_recent_base_transform_path()
    if base_transform_file is None:
        print('No transform between world/pattern and robot base frame available, make sure the eye-in-hand calibration is up to date.')
        return
    
    t = calibration_io.load_transform_from_yaml(base_transform_file) # the transform from the robot base to the world frame, or the pose of the robot base expressed in the world/pattern frame
    
    all_imgs = sorted([f for f in os.listdir(im_dir) if f.endswith('.JPG')])
    indices = np.asarray([np.where(np.array(all_imgs)==name) for name in used]).flatten()
    cam_in_base = poses[indices,:] # [x,y,z,theta_x,theta_y,theta_z]

    # multiply (transform from camera to robot base frame) x (transform from robot base to world frame) to get the (transform from camera to world frame)
    cam_in_base_mat = conversion.robot_poses_as_htms(cam_in_base) # convert to homogeneous transformation matrices
    cam_poses_from_robot = t @ cam_in_base_mat # poses of the camera, given in the pattern frame, as derived from the robot's proprioception measurements
    cam_poses_from_images = cam_in_pattern # poses of the camera, given in the pattern frame, as derived from the AprilTag locations in the images

    try:
        plotting.plot_transforms(cam_poses_from_images)
    except:
        print('Could not plot transforms, use Python 3.10 and install Pytransform3D to show these plots.')
    try:
        plotting.plot_transforms(cam_poses_from_robot)
    except:
        print('Could not plot transforms, use Python 3.10 and install Pytransform3D to show these plots.')

    diff = np.subtract(cam_poses_from_robot, cam_poses_from_images)

    rot_err = np.asarray([np.linalg.norm(cv2.Rodrigues(cam_poses_from_robot[i,:3,:3].dot(cam_poses_from_images[i,:3,:3].T))[0]) for i in range(diff.shape[0])]) # in radians
    rot_err2 = np.asarray([np.rad2deg(np.arccos((np.trace(cam_poses_from_robot[i,:3,:3] @ cam_poses_from_images[i,:3,:3].T) - 1) / 2)) for i in range(diff.shape[0])]) # in degrees
    # np.testing.assert_almost_equal(np.rad2deg(rot_err), rot_err2) # you can verify that these are in fact the same
    # methods from https://stackoverflow.com/questions/6522108/error-between-two-rotations
    # and https://math.stackexchange.com/questions/2113634/comparing-two-rotation-matrices

    # save results
    tl_err = diff[:,:3,3]
    with open(os.path.join(im_dir, 'translation_errors.npy'), 'wb') as f:
        np.save(f, diff[:,:3,3])
    with open(os.path.join(im_dir, 'rotation_errors.npy'), 'wb') as f:
        np.save(f, rot_err2)
    plot_pose_errors(tl_err, rot_err2)
    return

def plot_pose_errors(tl_err, rot_err):
    max_err = np.ceil(np.max(np.max(abs(tl_err), axis=0))*1000)/1000

    fig = plt.figure()
    ax1 = fig.add_subplot(2,2,1)
    x_errors = plt.hist(tl_err[:,0], bins=20)
    plt.xlabel('Error [m]')
    plt.title('X errors')
    plt.xlim(-max_err,max_err)

    ax2 = fig.add_subplot(2,2,2)
    y_errors = plt.hist(tl_err[:,1], bins=20)
    plt.xlabel('Error [m]')
    plt.title('Y errors')
    plt.xlim(-max_err,max_err)

    ax3 = fig.add_subplot(2,2,3)
    z_errors = plt.hist(tl_err[:,2], bins=20)
    plt.xlabel('Error [m]')
    plt.title('Z errors')
    plt.xlim(-max_err,max_err)

    ax4 = fig.add_subplot(2,2,4)
    plt.hist(rot_err, bins=20)
    plt.xlabel('Error [°]')
    plt.title('Rotation errors')

    print(f'Max errors: {np.max(abs(tl_err[:,0]))} meters in x, {np.max(abs(tl_err[:,1]))} meters in y, {np.max(abs(tl_err[:,2]))} meters in z, and {np.max(rot_err)} degrees in rotation')
    print(f'Mean errors: {np.mean(abs(tl_err[:,0]))} meters in x, {np.mean(abs(tl_err[:,1]))} meters in y, {np.mean(abs(tl_err[:,2]))} meters in z, and {np.mean(rot_err)} degrees in rotation')

    plt.show()

def plot_pose_errors_by_repeat(tl_err, rot_err, repeats=4):
    set_length = int(tl_err.shape[0]/repeats)
    fig = plt.figure()
    ax1 = fig.add_subplot(2,2,1)
    plt.title('Translation errors in x')
    plt.plot(tl_err[:set_length,0], label='10°/s')
    plt.plot(tl_err[set_length:2*set_length,0], label='20°/s')
    plt.plot(tl_err[2*set_length:3*set_length,0], label='30°/s')
    plt.plot(tl_err[3*set_length:,0], label='40°/s')
    plt.xlabel('Pose Nr. along the path')
    plt.ylabel('Error [m]')
    plt.axhline(0, color='r')
    plt.legend()

    ax2 = fig.add_subplot(2,2,2)
    plt.plot(tl_err[:set_length,1], label='10°/s')
    plt.plot(tl_err[set_length:2*set_length,1], label='20°/s')
    plt.plot(tl_err[2*set_length:3*set_length,1], label='30°/s')
    plt.plot(tl_err[3*set_length:,1], label='40°/s')
    plt.title('Translation errors in y')
    plt.xlabel('Pose Nr. along the path')
    plt.ylabel('Error [m]')
    plt.axhline(0, color='r')
    plt.legend()

    ax3 = fig.add_subplot(2,2,3)
    plt.plot(tl_err[:set_length,2], label='10°/s')
    plt.plot(tl_err[set_length:2*set_length,2], label='20°/s')
    plt.plot(tl_err[2*set_length:3*set_length,2], label='30°/s')
    plt.plot(tl_err[3*set_length:,2], label='40°/s')
    plt.title('Translation errors in z')
    plt.xlabel('Pose Nr. along the path')
    plt.ylabel('Error [m]')
    plt.axhline(0, color='r')
    plt.legend()

    ax4 = fig.add_subplot(2,2,4)
    plt.plot(rot_err[:set_length], label='10°/s')
    plt.plot(rot_err[set_length:2*set_length], label='20°/s')
    plt.plot(rot_err[2*set_length:3*set_length], label='30°/s')
    plt.plot(rot_err[3*set_length:], label='40°/s')
    plt.title('Rotation errors')
    plt.xlabel('Pose Nr. along the path')
    plt.ylabel('Error [°]')
    plt.legend()
    plt.show()
    return

def plot_errors_from_files(im_dir):
    with open(os.path.join(im_dir, 'translation_errors.npy'), 'rb') as f:
        tl_err = np.load(f)
    with open(os.path.join(im_dir, 'rotation_errors.npy'), 'rb') as f:
        rot_err = np.load(f)
    plot_pose_errors(tl_err, rot_err)
    plot_pose_errors_by_repeat(tl_err, rot_err, repeats=4)

    # and again split by speed
    speeds = [10,20,30,40]
    set_length = int(tl_err.shape[0]/len(speeds))
    for i,speed in enumerate(speeds):
        plot_pose_errors(tl_err[i*set_length:(i+1)*set_length], rot_err[i*set_length:(i+1)*set_length])
    return


def main(cam_id='EOS01'):
    '''
    Ideally run this first part using Python 3.8 - the Kinova Kortex API has not been updated beyond that yet.
    '''
    # Take measurements
    #im_dir = set_vs_measured_states()
    # alternatively specify a previous measurement set
    im_dir = '/home/kh790/data/test_measurements/set_vs_measured_states/2024-03-18_15-31-43'


    '''
    Ideally, run this part using Python 3.10
    Some plots won't display if using earlier versions of Python, because the Pytransform3D library is not compatible with earlier versions.
    '''
    # analyse_joint_errors(im_dir)
    # analyse_pose_errors(im_dir, cam_id)
    plot_errors_from_files(im_dir)

if __name__ == "__main__":
    main()