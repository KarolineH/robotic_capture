#!/usr/bin/env python3.8

import os
import yaml
import pathlib

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io

''' 
Routine for calibrating the robot-mounted camera (intrinsics).
This should be performed ideally every time the system is started up, but at least every time after relevant camera or lens parameters have been changed.

Place a AprilTag calibration pattern down in the robot's workspace, and make sure the camera can see it from all angles.
This routine will move the robot through a pre-defined routine record either a rapid series full-resolution images or the HDMI video feed along the way.

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
If using HDMI feed, the camera should be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''

def record_data(capture_params=[32,'AUTO','AUTO',True], use_hdmi_stream = False, hdmi_device = '/dev/video2', output_directory='/home/kh790/data/calibration_imgs/cam_calib'):
    '''
    Move the robot, capture full-res still images OR 1080p HDMI video for camera calibration.
    Robot/wrist/camera poses are not recorded for this.
    '''

    # TODO: make sure the image directory exists and is empty!!

    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is enabled, you can instead add a focus operation before each capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)
    if use_hdmi_stream:
        rec = Recorder(hdmi_device) # instantiate the HDMI video recorder object

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        # TODO: Improve the motion sequence. Look also at Waypoints to achieve a smooth motion.
        sequence, action_list = data_io.read_action_from_file(actions_dir + "/DSLR_example_path.json")
        for i,state in enumerate(action_list[:4]):
            IF.execute_action(state) # the first 4 poses are just for the robot to reach the starting position

        # Start recording when the robot reaches the correct state
        if use_hdmi_stream:
            # EITHER start recording HDMI output stream to a file
            rec.start_recording(os.path.join(output_directory, 'video' ,'camera_calibration.mp4'))
        else:
            # OR start capturing a burst (=rapid series) of full-resolution still images
            burst_running = cam.start_burst(speed=0)

        # Execute the rest of the movement sequence
        for i,state in enumerate(action_list[4:]):
            IF.execute_action(state)

        # And stop the recording/capture
        if use_hdmi_stream:           
            rec.stop_recording()
            # TODO: Convert into single frames/images
        else:
            sucess, files, msg =cam.stop_burst()
            for path in files:
                # Files need to be downloaded here, which might take a while
                cam.download_file(path, target_file=os.path.join(output_directory,path.split('/')[-1]))
    return

def calibrate(calib_file='./camera_info.yaml', im_dir='/home/kh790/data/calibration_imgs/cam_calib', save=True):
    '''
    Perform calibration using the images in im_dir.
    This is done via AprilTag detection and OpenCV camera calibration.
    '''

    cc = CamCalibration('mounted_camera', im_dir)
    frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True)
    calibration_io.save_to_yaml(calib_file, cc.name, frame_size, matrix, distortion)
    return frame_size, matrix, distortion, cam_in_world, used

if __name__ == "__main__":
    #record_data(use_hdmi_stream=True)
    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config'
    target_file = calibr_dir + '/camera_info.yaml'
    __, matrix, distortion, __, __ = calibrate(target_file)