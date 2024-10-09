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

def set_vs_measured_states(output_dir, focus_dist=10, sequence_file='/home/kh790/data/paths/intrinsics_calib.txt', capture_params=[22,'AUTO','AUTO',False]):
    '''
    Move through a sequence of joint states
    Measures the joint angles repeatedly immediately after reaching the state
    Also takes a full-res image of the AprilTag pattern at each state
    Saves the error in measured joint angles compared to the set joint angles and the measurement's time stamps to file
    Also saves the measured cartesian camera pose to file (in the robot's base frame)

    This recording routine can take around 30 minutes to complete, depending on the number of states, waiting times, and range of tested speeds.
    Works best when continuous autofocus is enabled and the auto focus mode is set to face tracking.
    '''

    num_measurements = 40 # number of pose measurements to take after arriving at each state
    sleep_time = 5 # seconds

    # create new directory for the images and measurements
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = f'{output_dir}/{stamp}'
    os.mkdir(im_dir)

    # load the movement sequence
    states = np.loadtxt(sequence_file, delimiter=',') # a pre-recorded path of joint states loaded from file
    sequence = [data_io.create_angular_action(np.asarray(goal)) for goal in states]

    actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
    rest_pose = data_io.read_action_from_file(actions_dir + "/standby_pose.json")
    ready_pose = data_io.read_action_from_file(actions_dir + "/ready_pose.json")

    # prep the camera
    cam = EOS()
    cam.set_capture_parameters(*capture_params)
    set_focus(cam, focus_dist)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe start pose
        success &= IF.execute_action(ready_pose) # reach the starting position

        errors = []
        timestamps = []
        poses = []

        # Reach a series of joint states
        for i,state in enumerate(sequence):
            errors_by_state = []# [measurements x joints]
            timestamps_by_state = []
        
            target = [joint.value for joint in list(state.reach_joint_angles.joint_angles.joint_angles)]
            IF.execute_action(state)
            start = time.time()
            for j in range(num_measurements):
                timestamps_by_state.append(time.time() - start)
                measured_state = IF.get_joint_angles() # measure the reached joint angles
                diff = np.subtract(measured_state, target) # calculate the error between the measured and target joint angles
                errors_by_state.append(diff)

            errors.append(errors_by_state) # [states x measurements x joints]
            timestamps.append(timestamps_by_state)

            time.sleep(sleep_time) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
            cam_pose = IF.get_pose() # now measure the pose (coordinates) of the camera after settling
            poses.append(cam_pose)
            path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

        success &= IF.execute_action(ready_pose)

        errors = np.asarray(errors) # shape [states,measurements,joints]
        timestamps = np.asarray(timestamps) # shape [states,measurements,joints]
        poses = np.asarray(poses) # shape [states,6], each being the camera pose [x,y,z,theta_x,theta_y,theta_z]

        with open(os.path.join(im_dir, 'state_errors.npy'), 'wb') as f:
            np.save(f, errors)
        with open(os.path.join(im_dir, 'state_timestamps.npy'), 'wb') as f:
            np.save(f, timestamps)
        np.savetxt(os.path.join(im_dir, 'measured_cam_poses.txt'), poses, delimiter=',')
    return im_dir

def set_focus(cam, focus_dist):
    for i in range(17):
        cam.manual_focus(value=2) # bring the focus gradually to the near point for a fixed reference point
        time.sleep(0.25)
    for i in range(focus_dist):
        cam.manual_focus(value=6) # focus manually to the desired distance as specified in nr. of large steps
        time.sleep(0.25)
    print(f"Focus set to {focus_dist} steps from near limit.")
    return

def analyse_joint_errors(im_dir):
    '''
    Plot the errors in the measured joint angles compared to the set joint angles recorded in set_vs_measured_states
    '''

    with open(os.path.join(im_dir, 'state_errors.npy'), 'rb') as f:
        errors = np.load(f)
    with open(os.path.join(im_dir, 'state_timestamps.npy'), 'rb') as f:
        timestamps = np.load(f)

    # Dimensions of the error array
    num_lines, num_x_values, num_subplots = errors.shape
    plt.figure()
    for subplot_idx in range(num_subplots):
        if subplot_idx == 0:
            ax1 = plt.subplot(2, 3, 1)
        else:
            plt.subplot(2, 3, subplot_idx +1, sharey=ax1)
        for line_idx in range(num_lines):
            plt.plot(timestamps[line_idx], errors[line_idx, :, subplot_idx], label=f'Line {line_idx + 1}')
        plt.title(f'Joint nr. {subplot_idx + 1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Error [°]')
        plt.ylim(-0.2, 0.2)
        plt.xlim(0, 0.8)

    # Show the plots
    plt.savefig(os.path.join(im_dir, f'joint_errors.png')) # save to png
    plt.show()
    return

def analyse_pose_errors(im_dir, cam_id):
    '''
    Compares camera poses recorded by the robot arm with camera poses derived from tags in images. 
    Computation time scales with the number of images, this might take a while.
    '''
    # read file
    poses = np.loadtxt(os.path.join(im_dir, 'measured_cam_poses.txt'), delimiter=',') # [n,6]
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
    cam_in_base = poses[sorted(indices),:] # [x,y,z,theta_x,theta_y,theta_z]
    cam_in_base_mat = conversion.robot_poses_as_htms(cam_in_base) # convert to homogeneous transformation matrices
    result1 = analyse_relatvive_tf_errors(cam_in_base_mat, cam_in_pattern)

    # multiply (transform from camera to robot base frame) x (transform from robot base to world frame) to get the (transform from camera to world frame)
    cam_poses_from_robot = conversion.invert_transform(t) @ cam_in_base_mat # poses of the camera, given in the pattern frame, as derived from the robot's proprioception measurements
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

    tl_err = diff[:,:3,3]
    # save results
    with open(os.path.join(im_dir, 'translation_errors.npy'), 'wb') as f:
        np.save(f, diff[:,:3,3])
    with open(os.path.join(im_dir, 'rotation_errors.npy'), 'wb') as f:
        np.save(f, rot_err2)
    plot_pose_errors(tl_err, rot_err2, im_dir)
    return

def relative_tf_between_poses(T1, T2):
    # Inverse of the first pose
    T1_inv = np.linalg.inv(T1)
    # Relative transform from T1 to T2
    relative_transform = np.dot(T1_inv, T2)
    return relative_transform

def analyse_relatvive_tf_errors(robot_tfs, tag_tfs):
    robot_relative = np.asarray([relative_tf_between_poses(robot_tfs[i], robot_tfs[0]) for i in range(len(robot_tfs)-1)])
    tag_relative = np.asarray([relative_tf_between_poses(tag_tfs[i], tag_tfs[0]) for i in range(len(tag_tfs)-1)])
    diff = robot_relative - tag_relative
    translation_axes_avg_error = np.mean(diff[:,:3,3], axis=0)
    translation_euclid_avg_error = np.mean([np.linalg.norm(diff[i,:3,3]) for i in range(diff.shape[0])])

    rotation_diff = np.asarray([robot_relative[i,:3,:3].dot(tag_relative[i,:3,:3].T) for i in range(diff.shape[0])])
    rotation_avg_error = [np.linalg.norm(cv2.Rodrigues(rotation_diff[i])[0]) for i in range(diff.shape[0])]
    np.mean(rotation_avg_error, axis=0)

    trace = np.asarray([np.trace(rotation_diff[i]) for i in range(diff.shape[0])])
    trace[np.where(trace>3.0)] = 3.0
    trace[np.where(trace<-1.0)] = -1.0
    angle_diff = np.arccos((trace - 1.0) / 2.0)
    # Convert angle to degrees
    angle_diff_deg = np.degrees(angle_diff)
    print(f'Average translation error along the axes: {translation_axes_avg_error}')
    print(f'Max translation error along the axes: {np.max(diff[:,:3,3], axis=0)}')
    print(f'Average translation error in Euclidean space: {translation_euclid_avg_error}')
    print(f'Max euclidean translation error: {np.max([np.linalg.norm(diff[i,:3,3]) for i in range(diff.shape[0])])} meters')
    print(f'Average rotation error: {np.mean(angle_diff_deg)} degrees')
    print(f'Max rotation error: {np.max(angle_diff_deg)} degrees')
    return True

def plot_pose_errors(tl_err, rot_err, im_dir):
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

    plt.savefig(os.path.join(im_dir, f'pose_errors.png')) # save to png
    plt.show()

def main(out_dir='/home/kh790/data/tests', record=False, cam_id='EOS01', lens_id=0):

    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config' # default location is the config directory of this package
    __, min_aperture, focus_dist = calibration_io.load_lens_config(calibr_dir + '/lens_config.yaml', lens_id)


    # Take measurements
    if record:
        image_path = set_vs_measured_states(out_dir, focus_dist, capture_params=[min_aperture,'AUTO','AUTO',False])
    else:
        # If the images have already been captured, set the path to the directory containing the images
        image_path = out_dir


    analyse_joint_errors(image_path)
    analyse_pose_errors(image_path, cam_id)

if __name__ == "__main__":

    #out_dir = '/home/kh790/data/tests'
    out_dir = '/home/kh790/data/tests/2024-09-11_14-59-48'
    record = False

    main(out_dir, record)