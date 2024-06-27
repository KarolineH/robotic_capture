# eos_camera_if
This is a python interface for remote controlling one or multiple Canon EOS R5 C cameras.
Includes I/O control via the Camera's USB port (cam_io.py) as well as an additional passive recording channel from the camera's HDMI output (recording.py).

# Set-up instructions
1. Install the requirements (listed below).
2. Make sure all cameras are equipped with charged batteries.
3. Confirm that all cameras are equipped with SD cards.\
If the card is new or was previously formatted (initialized) by another camera or computer, it is advised to format the card using the camera's own menu. 
4. Turn on all cameras by setting the switches to PHOTO or VIDEO mode. Please note that many functions are only available in one of the two modes.

#### To use the USB I/O control
5. Connect all cameras to the PC via USB.
6. To check if everything is ready, open a terminal and run 'gphoto2 --auto-detect'\
If any of the cameras are not detected, please refer to gphoto2 documentation.
7. We also advise to turn the 'Auto rotate' feature OFF in the camera menu (on the camera itself) and we mostly use the 'Scene Intelligent Auto' or 'Flexible-priority AE' shooting modes. Please refer to the camera manual for details.

#### To use the passive HDMI recording
8. Connect your camera via HDMI to an HDMI capture device. Then connect the capture device to your PC.\
This is necessary to convert the HDMI-out signal. We use the startech.com 4K30-HDMI-CAPTURE device, which works natively in Ubuntu. Otherwise follow the manufacturer's recommendation for setup, drivers etc. 
9. Make sure HDMI output has not been disabled in the camera settings (see camera manual for details).
10. To test your connection to the device try the following:
    - `v4l2-ctl --list-devices`, find your HDMI capture device in the list, e.g. /dev/video2
    - `ffmpeg -f v4l2 -i /dev/video2 -vf "format=yuv420p" -f sdl "HDMI Capture"` to display the live stream and test if the connection is live

# Quick start
After following the set-up instructions, take a look at io_usage_examples.py, especially the top-level API calls: get_capture_parameters(), capture_image(), capture_video(), and show_live_preview().
For HDMI recording, recording.py has some usage examples at the end of the file.
multi_camera_rec.py offers a class for synchronising the recording of videos with multiple EOS cameras. Usage examples are at the bottom of the file.

# Camera Mode selection
- Use PHOTO mode for capturing stills and also for maximum control over capture parameters, including ISO and autofocus 
    - capture photos at full resolution
    - capture photo bursts at ~ 9 fps
    - a live preview video stream can be displayed with 960x640 resolution at around 15fps
    - recording videos to file is possible at 1024x576 and ~25 fps OR at 960x640 and ~60 fps
- Use VIDEO mode for full-resolution video capture
    - With suitable storage media, the camera supports formats up to 8192x4320 and 60fps, please refer to the camera user guide for more details
    - Note that this mode is not supported by older versions of gphoto2

# Requirements:
- [gphoto2 >= 2.5.27, libphoto2 >= 2.5.31](http://www.gphoto.org/doc/manual/index.html) (We recommend using this [gphoto2-updater tool](https://github.com/gonzalo/gphoto2-updater) for installation)\
- [python-gphoto2 v2.5.1](https://github.com/jim-easterbrook/python-gphoto2)
- [ffmpeg](https://ffmpeg.org/) `sudo apt-get install ffmpeg`
- Install python modules from requirements.txt

Tested with Python 3.10.12