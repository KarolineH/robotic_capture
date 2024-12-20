import os
import pathlib
import datetime
import time
import cv2

import yaml
import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
from eos_camera_if.cam_io import EOS
from eos_camera_if.recording import Recorder
from calibration.calibrate import CamCalibration
from calibration.helpers import calibration_io as calibration_io

''' 
Routine for calibrating the robot-mounted camera (intrinsics).
This should be performed ideally every time the system is booted up, but at least after any relevant camera or lens parameters have been changed.

Attach an AprilTag calibration pattern in the robot's workspace, make sure it can not move and the camera can see it from all angles.
This routine will move the robot through a pre-defined routine and record either a rapid series full-resolution images or the HDMI video feed along the way.
It is not advised to do both at the same time but you can run the routine twice if you need both types of data.

Before running, make sure the camera and robot are both connected, powered on, and ready to receive commands.
If using the HDMI feed, the camera should be connected using both the USB and HDMI port.
The robot should be in a safe resting position at the beginning. Please check that the movement routine is safe before running.
'''

def record_data(capture_params=[22,'AUTO','AUTO',False], focus_dist=None, use_hdmi_stream = False, burst=False, hdmi_device = '/dev/video2', sleep_time=2, output_directory='/home/kh790/data/calibration_imgs/intrinsics'):
    '''
    Move the robot, capture full-res still images OR 1080p HDMI video for camera calibration.
    Robot/wrist/camera poses are not recorded for this.

    Input parameters:
    capture_params: list of camera capture parameters. These are by default set to minimise Bokeh effects (select smallest aperture possible for the attached lens). Continuous atuofocus is disabled.
    use_hdmi_stream: if True, the HDMI video stream is recorded, If False, a series of full-resolution still images is captured.
    burst: if True, a burst of full-resolution still images is captured continuously during the movement sequence, if False single stills are captured when each defined state in the sequence is reached.
    hdmi_device: the device name of the HDMI capture device, by default this is set to /dev/video2 (not needed if use_hdmi_stream is False)
    sleep_time: the time in seconds to wait after each robot movement before capturing an image, to allow the arm to settle and avoid any vibration or mechanical inaccuracies.
    output_directory: the directory where the images or video will be saved. A sub-directory will be created at the location for each new calibration run.

    Returns the directory where the images or video are saved.
    '''

    # Prepare the image/video output directory
    if not os.path.exists(output_directory):
        print(f"Image output directory {output_directory} does not exist. Please specify an existing location and try again.")
        return None
    else:
        # create a new sub-directory at the target location so that images from each run are kept separate
        stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        im_dir = os.path.join(output_directory, stamp)
        os.mkdir(im_dir)

    # instantiate the camera interface object
    # change capture settings if needed
    # by default this is set to a small aperture to reduce Bokeh effects
    # also continuous autofocus is disabled so that capture parameters do not change during the capture
    cam = EOS()
    cam.set_capture_parameters(*capture_params)

    # If a desired fixed manual focus distance is specified, set it here. Otherwise leave the setting as previously defined or adjusted by hand.
    if focus_dist is not None:
        set_focus(cam, focus_dist) # set the focus to a fixed value

    if use_hdmi_stream:
        rec = Recorder(hdmi_device) # instantiate the HDMI video recorder object

    # Create connection to the robot
    args = k_util.parseConnectionArguments()
    with k_util.DeviceConnection.createTcpConnection(args) as router:
        IF = Kinova3(router)
        success = True

        # double check that the robot starts out in its safe resting position
        actions_dir = str(pathlib.Path(__file__).parent.resolve()) + '/kinova_arm_if/actions'
        sequence, action_list = data_io.read_action_from_file(actions_dir + "/orienting the robot's new workspace for caliibration.json") # this sequence has no start or stop positions integrated
        rest_position = data_io.read_action_from_file(actions_dir + "/standby_pose.json")
        ready_position = data_io.read_action_from_file(actions_dir + "/ready_pose.json")

        IF.execute_action(ready_position) # reach the starting position

        # Start recording once the robot reaches the starting state
        if use_hdmi_stream:
            # EITHER start recording HDMI output stream to a file
            rec.start_recording(os.path.join(im_dir,'camera_calibration_feed.mp4'))
        elif burst:
            # OR start capturing a burst (=rapid series) of full-resolution still images
            cam.trigger_AF(duration=1)
            active_burst = cam.start_burst(speed=0)

        # Execute the rest of the movement sequence
        for i,state in enumerate(action_list):
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
        IF.execute_action(ready_position)
        IF.execute_action(rest_position)
        print("Data recording done. Images saved in: ", im_dir)
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

def video_to_frames(vid_dir=None, sampling_rate=10):
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
    return

def calibrate(cam_id, calib_file='./camera_info.yaml', im_dir='/home/kh790/data/calibration_imgs/cam_calib', cam_model='OPENCV', save=True):
    '''
    Perform calibration using the images in im_dir.
    This is done via AprilTag detection and OpenCV camera calibration.
    Assumes the standard AprilTag pattern used in our lab, can be altered if needed, see the calibration module for details.
    '''
    print(f"Calibrating camera")
    cc = CamCalibration(cam_id, im_dir)
    frame_size,matrix,distortion,cam_in_world,used = cc.april_tag_calibration(lower_requirements=True, cam_model=cam_model)
    if save:
        calibration_io.save_intrinsics_to_yaml(calib_file, cam_id, cam_model, frame_size, matrix, distortion)
    print(f"Camera calibration done. Calibration parameters saved in: {calib_file}")
    return frame_size, matrix, distortion, cam_in_world, used

def main(cam_id='EOS01', lens=0, cam_model='OPENCV', record=False):

    # Find location for calibration outcomes and camera details
    calibr_dir = str(pathlib.Path(__file__).parent.resolve()) + '/config' # default location is the config directory of this package
    
    #First, run the data recording routine. 
    #Please be careful, this is a potentially dangerous operation. 
    #Be aware of your surroundings. The robot has no collision detection or obstacle awareness in this mode.
    #Adjust the output_directory if needed
    if record:

        __, min_aperture, focus_dist = calibration_io.load_lens_config(calibr_dir + '/lens_config.yaml', lens_id=lens)
        im_dir = record_data(capture_params=[min_aperture,'AUTO','AUTO',False], focus_dist=focus_dist)
        if im_dir is None:
            return
    else:
        #alternatively use a previously recorded set of images, specify the directory here
        im_dir = '/home/kh790/data/calibration_imgs/intrinsics/2024-10-31_17-34-33'


    # Then use the recorded data to calibrate the camera intrinsics
    # Specify a target file, where the calibration parameters will be saved
    # By default this is named by the timestamp assigned to the image directory, optionally specify a different location here
    stamp = im_dir.split('/')[-1]
    target_file = calibr_dir + f'/camera_info_{cam_id}_{stamp}_{cam_model}.yaml'

    # Perform the calibration
    __, matrix, distortion, __, __ = calibrate(cam_id, calib_file=target_file, im_dir=im_dir, cam_model=cam_model)

if __name__ == "__main__":
    # camera model options: 'OPENCV', 'SIMPLE_PINHOLE', 'FULL_OPENCV'
    # lens options 0=28mm lens (default), 1=70mm lens
    main(cam_id='EOS01', lens=0, cam_model = 'SIMPLE_PINHOLE', record=False)