import numpy as np
import pathlib
import os
from kinova_arm_if.helpers import conversion as conv
from calibration.helpers import calibration_io as calibio
from calibration.helpers import plotting

def wrist_to_cam_poses(in_file):
    '''
    Transforms robot wrist poses to camera poses. 
    Fetches the latest eye-in-hand calibration results and applies the transformation.
    '''

    config_dir = str(pathlib.Path(__file__).parent.absolute()) + '/config'
    latest_tf_calib_file= os.path.join(config_dir, sorted([entry for entry in os.listdir(config_dir) if 'dslr_transform' in entry])[-1])
    latest_cam_tf = calibio.load_transform_from_yaml(latest_tf_calib_file)

    raw_poses = np.loadtxt(in_file, delimiter=',')
    wrist_tfs = conv.robot_poses_as_htms(raw_poses)

    cam_tfs = np.matmul(wrist_tfs, latest_cam_tf)


    inv_transforms = np.asarray([conv.invert_transform(mat) for mat in cam_tfs]) # invert the transforms
    Rs = inv_transforms[:, :3, :3]
    ts = inv_transforms[:, :3, 3]
    quaternions = conv.mat_to_quat(Rs)
    quaternions = np.concatenate((quaternions[:,3].reshape(-1,1), quaternions[:,:3]), axis=1)

    # 3. Formatting and saving to txt file
    with open('/home/karo/Downloads/scan13/fixed_poses.txt', 'w') as f:
        f.write("# id, QW, QX, QY, QZ, TX, TY, TZ, camera_id\n")
        for i in range(len(quaternions)):
            # COLMAP expects image_id, qw, qx, qy, qz, tx, ty, tz, camera_id, file name
            f.write(f"{i+1} {quaternions[i][0]} {quaternions[i][1]} {quaternions[i][2]} {quaternions[i][3]} {ts[i][0]} {ts[i][1]} {ts[i][2]} 1 IMG_{6807+i}.JPG \n \n")



if __name__ == '__main__':
    wrist_to_cam_poses('/home/karo/Downloads/scan13/raw_poses.txt')