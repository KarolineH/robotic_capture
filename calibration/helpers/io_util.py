import numpy as np
import yaml

def save_to_yaml(out_file, name, size, k, d):
    assert k.shape == (3, 3)
    calibration_data = {
        "image_width": size[0],
        "image_height": size[1],
        "camera_name": name,
        "camera_matrix": np.around(k, decimals=5).tolist(),
        "distortion_model": 'OPENCV',
        "distortion_coefficients": np.around(d, decimals=5).tolist()[0]
    }
    
    with open(out_file, 'w') as of:
        yaml.dump(calibration_data, of)
    return

def load_from_yaml(in_file):
    with open(in_file, 'r') as f:
        calib = yaml.safe_load(f)

    name = calib['camera_name']
    k = np.asarray(calib['camera_matrix'])
    d = np.asarray(calib['distortion_coefficients'])
    size = (calib['image_width'], calib['image_height'])
    return name, size, k, d

if __name__ == "__main__":
    # Test load_from_yaml
    calib = load_from_yaml('calibration.yaml')
    print(calib)