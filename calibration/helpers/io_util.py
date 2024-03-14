import numpy as np
import yaml


def save_to_yaml(out_file, cam_name, frame_size, matrix, distortion):

    '''
    Save camera instrinsics to a yaml file. 
    '''
    
    assert matrix.shape == (3, 3)
    data = {
        'image_width': frame_size[0],
        'image_height': frame_size[1],
        'camera_name': cam_name,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': matrix.flatten().tolist()
        },
        'distortion_model': 'OPENCV',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': distortion.flatten().tolist()
        }
    }
    yaml.dump(data, open(out_file, 'w'), sort_keys=False)
    return

def load_from_yaml(in_file):
    '''
    Load camera instrinsics from a yaml file.
    '''
    with open(in_file, 'r') as f:
        calib = yaml.safe_load(f)
    name = calib['camera_name']
    matrix = np.asarray(calib['camera_matrix']['data']).reshape([3,3])
    distortion = np.asarray(calib['distortion_coefficients']['data'])
    frame_size = (calib['image_width'], calib['image_height'])
    return name, frame_size, matrix, distortion

def transform_to_yaml(out_file, data):
    '''
    Save a transformation matrix to a yaml file.
    '''
    assert data.shape == (4, 4)
    yaml.dump(data.tolist(), open(out_file, 'w'), sort_keys=False)
    return

def transform_from_yaml(in_file):
    '''
    Load a transformation matrix from a yaml file.
    '''
    with open(in_file, 'r') as f:
        data = yaml.safe_load(f)
    return np.asarray(data)

if __name__ == "__main__":
    # Test load_from_yaml
    calib = load_from_yaml('calibration.yaml')
    print(calib)