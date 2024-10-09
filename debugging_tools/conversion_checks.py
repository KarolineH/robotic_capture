import pathlib
import os
from calibration.helpers import calibration_io as calibio
from kinova_arm_if.helpers import conversion as conv
import numpy as np
from calibration.helpers import plotting

file1 = '/home/kh790/data/scans/2024-10-09_12-55-28/raw_poses.txt'
file2 = '/home/kh790/data/scans/2024-10-09_12-56-28/raw_poses.txt'


config_dir = str(pathlib.Path(__file__).parent.absolute()) + '/config'
latest_tf_calib_file= os.path.join(config_dir, sorted([entry for entry in os.listdir(config_dir) if 'dslr_transform' in entry])[-1])
latest_cam_tf = calibio.load_transform_from_yaml(latest_tf_calib_file)

recorded_wrist_poses = np.loadtxt(file1, delimiter=',')
recorded_camera_poses = np.loadtxt(file2, delimiter=',')

recorded_wrist_tfs = conv.robot_poses_as_htms(recorded_wrist_poses)
recorded_camera_tfs = conv.robot_poses_as_htms(recorded_camera_poses)

cam_tfs = np.matmul(recorded_wrist_tfs, latest_cam_tf)
plotting.plot_transforms(np.vstack([recorded_camera_tfs, cam_tfs]))
print("Done")