#!/usr/bin/env python3.8

import os
import matplotlib.pyplot as plt
import numpy as np
import time
import pathlib

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
import kinova_arm_if.helpers.conversion as conv


def set_vs_measured_states():
    test_speed_limits = [10,20,30,40]
    num_measurements = 40

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router, transform_path='./garbage.json')
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        errors_overall = []
        timesteps_overall = []

        for speed in test_speed_limits:
            joint_speeds = [speed] * 6
            IF.set_speed_limit(speeds=joint_speeds, control_mode=4)
            
            errors_by_speed = []
            timesteps_by_speed = []

            # Reach a series of joint states
            sequence, action_list = data_io.read_action_from_file(actions_dir + '/scan_path.json')
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

            errors_overall.append(errors_by_speed)
            timesteps_overall.append(timesteps_by_speed)
        
        errors = np.asarray(errors_overall) # shape [speeds,states,measurements,joints]
        timesteps = np.asarray(timesteps_overall) # shape [speeds,states,measurements,joints]

        with open('/home/kh790/data/test_measurements/states_errors.npy', 'wb') as f:
            np.save(f, errors)
        with open('/home/kh790/data/test_measurements/states_timesteps.npy', 'wb') as f:
            np.save(f, timesteps)
    return


def set_vs_measured_poses():
    # Reach a series cartesian coordinates
    #...
    return


def plot_errors(x, y, name):
    plt.figure()
    for joint_idx in range(y.shape[1]):
        plt.plot(x, y[:,joint_idx])
    plt.legend(['joint 1','joint 2', 'joint 3', 'joint 4', 'joint 5', 'joint 6'])
    plt.title('Measured Joint Angle Error after executing movement')
    plt.xlabel('time [seconds]')
    plt.ylabel('Joint Angle Error [°]')
    plt.savefig('figures/' + name)
    return

def pose_vs_apriltag():
    return



if __name__ == "__main__":
    set_vs_measured_states()