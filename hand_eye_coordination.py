#!/usr/bin/env python3.8

import sys
import os
import time
import json
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder

''' 
Routine for calibrating the hand-eye coordination between robot arm and mounted DSLR camera.
This should only need to be performed once, and again whenever the camera is moved or the dimensions of the camera, mount, or lens change.

This routine will record move the robot through 10 target states and record both the wrist poses and a full-resolution image in each state.
In parallel, you can optionally record a 1080p video feed from the camera throughout the whole movement. 

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
The camera should be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''


output_directory = '/home/kh790/data/hand_eye_coord'
camera = '/dev/video2' # change this to the correct video device if needed


def record_data(remote_control_cam = False, record_video = False):

    # INPUT: set to True if you want to control the camera focus and shutter remotely, False if you want to snap the pictures by hand

    IF = Kinova3() # get robot Python interface object, has all needed methods
    if remote_control_cam:
        cam = EOS() # instantiate the camera interface object
        #cam.set_capture_parameters(aperture='AUTO', iso='AUTO', shutterspeed='AUTO', c_AF=False) # change capture settings if needed
        for i in range(10):
            cam.manual_focus(2)
    if record_video:
        rec = Recorder(camera) # instantiate the HDMI video recorder object

    # Create connection to the robot and get the router
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        success = True

        # double check that the robot is in its safe resting position
        rest_action = data_io.read_action_from_file("./kinova3_if/rest_on_foam_cushion.json")
        success &= IF.execute_action(base, rest_action)

        example_sequence, action_list = data_io.read_action_from_file("./kinova3_if/hand_eye_sequence.json")
        poses = []

        if record_video:
            rec.start_recording(os.path.join(output_directory, 'hand_eye_calibration.mp4')) # start recording the robot's movement

        for i,state in enumerate(action_list):
            IF.execute_action(base, state)
            if i > 1 and i < len(action_list)-1:
                time.sleep(2) # wait longer here if the robot tends to shake/vibrate, to make sure an image is captured without motion blur and at the correct position
                wrist_pose = IF.get_pose(base_cyclic)
                poses.append(wrist_pose)
                if remote_control_cam:
                    #cam.trigger_AF() # trigger the camera to focus
                    path, msg = cam.capture_image(download=True, target_path=output_directory) # capture an image and download it to the specified directory
                    print(msg)
        if record_video:           
            rec.stop_recording()

        pose_data = [pose.tolist() for pose in poses]
        json.dump(pose_data, open(os.path.join(output_directory, 'hand_eye_wrist_poses.json'), 'w'))


def get_camera_poses(output_directory):
    return

def calibration():
    return

if __name__ == "__main__":
    record_data(remote_control_cam = True, record_video = False)
    # get_camera_poses(output_directory)
    # calibration()
    pass