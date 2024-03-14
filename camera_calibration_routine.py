#!/usr/bin/env python3.8
'''
Run this script using Python <= 3.8
'''

import os
import pathlib
import datetime
import time

import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder
from calibration.calibrate import CamCalibration
from calibration.helpers import io_util as calibration_io

''' 
Routine for calibrating the robot-mounted camera (intrinsics).
This should be performed ideally every time the system is booted up, but at least after any relevant camera or lens parameters have been changed.

Place an AprilTag calibration pattern down in the robot's workspace, and make sure the camera can see it from all angles.
This routine will move the robot through a pre-defined routine and record either a rapid series full-resolution images or the HDMI video feed along the way.
It is not advised to do both at the same time but you can run the routine twice if you need both types of data.

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
If using the HDMI feed, the camera should be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''

def record_data(capture_params=[32,'AUTO','AUTO',True], use_hdmi_stream = False, burst=False, hdmi_device = '/dev/video2', sleep_time=2, output_directory='/home/kh790/data/calibration_imgs/cam_calib'):
    '''
    Move the robot, capture full-res still images OR 1080p HDMI video for camera calibration.
    Robot/wrist/camera poses are not recorded for this.
    Input parameters:
    capture_params: list of camera capture parameters, these are by default set to minimise Bokeh effects (small aperture) and to facilitate better focus on the calibration pattern.
    use_hdmi_stream: if True, the HDMI video stream is recorded, If False, a burst of full-resolution still images is captured.
    burst: if True, a burst of full-resolution still images is captured continuously during the movement sequence, if False single stills are captured when each defined state in the sequence is reached.
    sleep_time: the time in seconds to wait after each robot movement before capturing an image, to allow the arm to settle.
    hdmi_device: the device name of the HDMI capture device, by default this is set to /dev/video2 (not needed if use_hdmi_stream is False)
    output_directory: the directory where the images or video will be saved. A sub-directory will be created at the location for each run.
    Returns the directory where the images or video are saved.
    '''

    # Prepare the image/video output directory
    if not os.path.exists(output_directory):
        print(f"Image output directory {output_directory} does not exist. Please specify an existing location and try again.")
        return
    else:
        # create a new sub-directory at the target location so that images from each run are kept separate
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        im_dir = os.path.join(output_directory, stamp)
        os.mkdir(im_dir)

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
        IF.set_speed_limit(joint_speeds=[20,20,20,20,20,20])
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        rest_action = data_io.read_action_from_file(actions_dir + "/rest_on_foam_cushion.json")
        success &= IF.execute_action(rest_action)

        sequence, action_list = data_io.read_action_from_file(actions_dir + "/calibration_sequence_20.json") # this sequence has 23 states, the first and last two are for reaching the starting and resting positions only
        for i,state in enumerate(action_list[:2]):
            IF.execute_action(state) # reach the starting position

        # Start recording once the robot reaches the starting state
        if use_hdmi_stream:
            # EITHER start recording HDMI output stream to a file
            rec.start_recording(os.path.join(im_dir,'camera_calibration_feed.mp4'))
        elif burst:
            # OR start capturing a burst (=rapid series) of full-resolution still images
            cam.trigger_AF(duration=1)
            active_burst = cam.start_burst(speed=0)
        else:
            # Capture the first still
            time.sleep(sleep_time) # wait for the arm to settle
            file_path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

        # Execute the rest of the movement sequence
        for i,state in enumerate(action_list[2:-2]):
            IF.execute_action(state)
            if not (use_hdmi_stream or burst):
                # If capturing still images, not a burst or HDMI stream, capture an image at each pose
                time.sleep(sleep_time) # wait for the arm to settle
                file_path, cam_path, msg = cam.capture_image(download=True, target_path=im_dir) # capture an image and download it to the specified directory

        # Finally, stop the recording/capture if needed
        if use_hdmi_stream:           
            rec.stop_recording() # stop the recording
            video_to_frames(im_dir) # convert mp4 video file into series of images
        else:
            sucess, files, msg =cam.stop_burst() # stop the capture
            for path in files:
                # Now files need to be downloaded from the camera storage to the PC, which might take a while
                # Files are named in ascending alpha-numeric order, so they can be sorted by name
                cam.download_file(path, target_file=os.path.join(im_dir,path.split('/')[-1]))

        # Return to the resting position
        IF.execute_action(action_list[-2])
        IF.execute_action(action_list[-1])
    return im_dir

def video_to_frames(vid_dir=None, sampling_rate=10):
    import cv2
    vidcap = cv2.VideoCapture(os.path.join(vid_dir, 'camera_calibration_feed.mp4'))
    success,image = vidcap.read()
    count = 0
    while success:
        if count % sampling_rate == 0:
            cv2.imwrite(os.path.join(vid_dir,f"frame{count}.jpg"), image)     # save frame as JPEG file      
            success,image = vidcap.read()
            print('Read a new frame: ', success)
        else:
            success,image = vidcap.read() # toss the frame
        count += 1

    # TODO: Convert into single frames/images
    return

def calibrate(calib_file='./camera_info.yaml', im_dir='/home/kh790/data/calibration_imgs/cam_calib', save=True):
    '''
    Perform calibration using the images in im_dir.
    This is done via AprilTag detection and OpenCV camera calibration.
    Assumes the standard AprilTag pattern used in our lab, can be altered if needed, see the calibration module for details.
    '''

    cc = CamCalibration('mounted_camera', im_dir)
    frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True)
    if save:
        calibration_io.save_to_yaml(calib_file, cc.name, frame_size, matrix, distortion)
    return frame_size, matrix, distortion, cam_in_world, used

if __name__ == "__main__":

    # First, run the data recording routine. Please be careful, this is a potentially dangerous operation. Be aware of your surroundings. The robot has no collision detection or obstacle awareness in this mode.
    #im_dir = record_data(use_hdmi_stream=True, burst=False, hdmi_device='/dev/video0')

    # Then use the recorded data to calibrate the camera (or optionally use a different set of images and skip the recording routine)
    # Specify a target file, where the calibration parameters will be saved
    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config' # default location is the config directory of this package
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S") # use the current date and time as a unique identifier, prevents overwriting previous calibrations
    target_file = calibr_dir + f'/camera_info_{stamp}.yaml'
    # Optionally specify a different image input directory
    #im_dir = '/home/karo/ws/data/calibration_images/cam_calib_test/' # you can also specify a different directory here
    __, matrix, distortion, __, __ = calibrate(calib_file=target_file, im_dir=im_dir)