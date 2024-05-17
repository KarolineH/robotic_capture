import numpy as np
import yaml
import pathlib
import os


def save_intrinsics_to_yaml(out_file, cam_id, model, frame_size, matrix, distortion):

    '''
    Save camera instrinsics to a yaml file. 
    '''

    # Find the number of distortion coefficients, this is the number of elements after which all remaining elements are zero.
    nr_dist_coeffs = np.max(np.where(distortion!=0)[1]) + 1
    
    assert matrix.shape == (3, 3)
    data = {
        'image_width': frame_size[0],
        'image_height': frame_size[1],
        'camera_name': cam_id,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': matrix.flatten().tolist()
        },
        'distortion_model': model,
        'distortion_coefficients': {
            'rows': 1,
            'cols': int(nr_dist_coeffs),
            'data': distortion.flatten()[:nr_dist_coeffs].tolist()
        }
    }
    yaml.dump(data, open(out_file, 'w'), sort_keys=False)
    return

def load_intrinsics_from_yaml(in_file):
    '''
    Load camera instrinsics from a yaml file.
    '''
    with open(in_file, 'r') as f:
        calib = yaml.safe_load(f)
    cam_id = calib['camera_name']
    cam_model = calib['distortion_model']
    matrix = np.asarray(calib['camera_matrix']['data']).reshape([3,3])
    distortion = np.asarray(calib['distortion_coefficients']['data'])
    frame_size = (calib['image_width'], calib['image_height'])
    return cam_id, frame_size, matrix, distortion, cam_model

def save_transform_to_yaml(out_file, data):
    '''
    Save a 4x4 homogeneous transformation matrix to a yaml file.
    '''
    assert data.shape == (4, 4)
    yaml.dump(data.tolist(), open(out_file, 'w'), sort_keys=False)
    return

def load_transform_from_yaml(in_file):
    '''
    Load a 4x4 homogeneous transformation matrix from a yaml file.
    '''
    with open(in_file, 'r') as f:
        data = yaml.safe_load(f)
    return np.asarray(data)

def fetch_recent_intrinsics_path(cam_id='mounted_camera'):
    '''
    Fetch the path of the most recent camera intrinsics from the config directory.
    If not available, return None.
    '''
    config_dir = str(pathlib.Path(__file__).parent.parent.parent.resolve()) + '/config' # this is the default location where camera intrinsics are saved
    matching_files = sorted([entry for entry in os.listdir(config_dir) if 'camera_info' in entry and cam_id in entry]) # find all calibration data for the specified camera ID
    if len(matching_files) > 0:
        most_recent_calib = matching_files[-1] # if there is one or more, select the most recent 
        cam_calib_file = os.path.join(config_dir, most_recent_calib) # get the full file path for the most recent calibration file
        if os.path.exists(cam_calib_file):
            # double check if file is available
            return cam_calib_file
        
    # if no file is available, return None
    return None

def fetch_recent_base_transform_path():
    '''
    Fetch the path of the most recent calibrated transform between the pattern origin (world) and the robot base.
    If not available, return None.
    '''
    config_dir = str(pathlib.Path(__file__).parent.parent.parent.resolve()) + '/config' # this is the default location where calibrated transfroms are saved
    matching_files = sorted([entry for entry in os.listdir(config_dir) if 'pattern2base' in entry]) # find all calibration data for the specified camera ID
    if len(matching_files) > 0:
        most_recent_calib = matching_files[-1] # if there is one or more, select the most recent 
        t_file = os.path.join(config_dir, most_recent_calib) # get the full file path for the most recent calibration file
        if os.path.exists(t_file):
            # double check if file is available
            return t_file
    
    # if no file is available, return None
    return None