import numpy as np
import pathlib
import os
import yaml

# Local imports
try:
    # Attempt a relative import
    from . import conversion as conv # if being run as a package
except ImportError:
    import conversion as conv # local case

def wrist_to_cam_poses(in_file):
    '''
    Transforms raw robot wrist poses to camera poses. 
    Fetches the latest eye-in-hand calibration results and applies the transformation.
    Input: A raw_poses.txt file containing robot wrist poses in the robot's pose convention (x,y,z,theta_x,theta_y,theta_z).
    Returns homogeneous transforms of the camera poses w.r.t the robot base frame.
    '''

    # fetch the most recent eye-in-hand calibration results
    config_dir = str(pathlib.Path(__file__).parent.parent.parent.absolute()) + '/config'
    latest_tf_calib_file= os.path.join(config_dir, sorted([entry for entry in os.listdir(config_dir) if 'dslr_transform' in entry])[-1])
    with open(latest_tf_calib_file, 'r') as f:
        data = yaml.safe_load(f)
        latest_cam_tf = np.asarray(data['transform'])

    raw_poses = np.loadtxt(in_file, delimiter=',')
    wrist_tfs = conv.robot_poses_as_htms(raw_poses)

    cam_tfs = np.matmul(wrist_tfs, latest_cam_tf)

    return cam_tfs

if __name__ == '__main__':
    pass
    #wrist_to_cam_poses('/home/kh790/data/scans/2024-10-09_12-56-28/raw_poses.txt')