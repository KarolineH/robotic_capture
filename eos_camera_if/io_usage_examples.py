
from cam_io import EOS
import helpers.gphoto_util as gphoto_util

### USAGE EXAMPLES ###

# Initialise camera
port = gphoto_util.choose_camera() # this is totally optional, useful if you have multiple cameras connected
cam1 = EOS(port=port) # if you don't specify a port, the first camera found will be used

# Get information about available cmaera configs
config_names = cam1.list_all_config() # list all
value, choices = cam1.get_config('autofocusdrive') # get details about a specific config

# Set a few parameters
current_value, choices, msg = cam1.set_image_format(list_choices=True) # what formats are available?
im_format,__,__ = cam1.set_image_format(0) # select the format
cam1.sync_date_time() # sync PC date and time to the camera
files = cam1.list_files() # see files stored on storage media in the camera
cam1.download_file(files[0], target_file='./test.jpg') # download a file from the camera to the PC

cam1.set_exposure_manual() # set exposure mode to manual, which allows remote manipulation of iso, aperture, and shutterspeed
msgs = cam1.set_capture_parameters(aperture=20, iso=120, shutterspeed='1/65', c_AF=False)

# msg = cam1.manual_focus(value=3) # focus manually
current_value, msg = cam1.set_AF_location(1000,500) # target a specific pixel location for AF
msg = cam1.trigger_AF()

# Capturing images and video
success, msg = cam1.capture_preview(target_file='./preview.jpg') # capture a preview image, i.e. the viewfinder display
cam1.show_live_preview() # start live preview, stop with q
out_file, msg = cam1.capture_image(download=True, target_file='./image.jpg') # capture a ful-res image
success, out_file, msg = cam1.capture_video(t=1, download=True, target_path='.') # capture a video, t is duration in seconds
success, files, msg = cam1.capture_burst(t=1) # capture a burst of images, t is duration in seconds

# And finally, record full-res video in VIDEO mode
success, file_path, msg = cam1.record_video(t=1, download=True, target_path='.')