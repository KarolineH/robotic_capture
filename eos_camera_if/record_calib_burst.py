from cam_io import EOS
import helpers.gphoto_util as gphoto_util
import os

'''
This is a useful script for manually recording some images for calibration purposes.
It sets the camera to burst mode and records a quick series of images. 
During the burst, autofocus is disabled and can not adjust, so make sure to position calibration pattern only within the bound of the depth of field.
Move the calibration pattern around in the camera's field of view to get a good spread of images. All images are saved in the target directory.
'''

target_dir = '/home/kh790/data/calibration_imgs/manual_calib/2024-05-30_14-08-50'
duration = 15

# Initialise camera
cam1 = EOS()# if you don't specify a port, the first camera found will be used
capture_params=[14,'AUTO','1/1000',True] # small aperture, fast shutter, to avoid Bokeh effects and motion blur
cam1.set_capture_parameters(*capture_params)

success, files, msg = cam1.capture_burst(t=duration) # capture a burst of images, t is duration in seconds
for path in files:
    # Now files need to be downloaded from the camera storage to the PC, which might take a while
    # Files are named in ascending alpha-numeric order, so they can be sorted by name
    cam1.download_file(path, target_file=os.path.join(target_dir,path.split('/')[-1]))

print("Data recording done. Images saved in: ", target_dir)