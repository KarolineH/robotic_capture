#!/usr/bin/env python3.8

import matplotlib.pyplot as plt
import numpy as np
import time
import pathlib
import json
import datetime
import os

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io
from eos_camera_if.cam_io import EOS

def set_vs_measured_states():
    test_speed_limits = [10,20,30,40]
    num_measurements = 40
    sleep_time = 3
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    im_dir = f'/home/kh790/data/test_measurements/set_vs_measured_states/{stamp}'
    os.mkdir(im_dir)
    actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
    sequence, action_list = data_io.read_action_from_file(actions_dir + '/calibration_sequence_20.json')

    capture_params=[32,'AUTO','AUTO',True]
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, transform_path='./garbage.json')
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
        json.dump(poses_unravelled, open(os.path.join(im_dir, 'hand_eye_wrist_poses.json'), 'w'))
    return

def anaylse():
    '''
    Show the error of the measured joint angles compared to the set joint angles for different speeds
    '''

    with open('/home/karo/Downloads/testing/states_errors.npy', 'rb') as f:
        errors = np.load(f)
    with open('/home/karo/Downloads/testing/states_timesteps.npy', 'rb') as f:
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

def pose_vs_apriltag():
    name, frame_size, matrix, distortion = calibration_io.load_from_yaml('/home/kh790/ws/robotic_capture/config/camera_info.yaml')
    cc = CamCalibration(name, '/home/kh790/data/test_measurements/set_vs_measured_states')
    __, __, __, cam_in_world, used_images = cc.april_tag_calibration(matrix, distortion, lower_requirements=True)

    with open('/home/karo/Downloads/testing//hand_eye_wrist_poses.json', 'r') as f:
        wrist_poses = json.load(f)

    # TODO: get the wrist poses from the file and compare
        
    # TODO: make sure the pattern is fixed and a transform from world to robot base is available
    return    


if __name__ == "__main__":
    #pose_vs_apriltag()
    set_vs_measured_states()
    anaylse()