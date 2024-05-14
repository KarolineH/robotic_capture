from scipy.spatial.transform import Rotation
import numpy as np

def euler_to_mat(theta_x=0, theta_y=0, theta_z=0):
    '''
    Convert euler angles to a 3x3 rotation matrix
    Assumes that the rotation is EXTRINSIC and applied in the order of x, y, z
    Also assumes that the angles are given in degrees (not rad)
    '''
    matrix = Rotation.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_matrix()
    return matrix

def mat_to_euler(matrix):
    '''
    Convert a 3x3 rotation matrix to euler angles
    Assumes that the rotation is EXTRINSIC and applied in the order of x, y, z
    Also returns the angles in degrees (not rad)
    '''
    euler = Rotation.from_matrix(matrix).as_euler('xyz', degrees=True)
    return euler

def euler_to_quat(theta_x=0, theta_y=0, theta_z=0):
    '''
    Convert euler angles to a quaternion
    Assumes that the Euler angle rotation is EXTRINSIC and applied in the order of x, y, z
    Also assumes that the angles are given in degrees (not rad)
    The output quaternion is given in scalar-last (x, y, z, w) format.
    '''
    quat = Rotation.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_quat()
    return quat

def mat_to_quat(matrix):
    '''
    Convert a 3x3 rotation matrix to a quaternion
    Returns the quaternion in scalar-last (x, y, z, w) format.
    '''
    quat = Rotation.from_matrix(matrix).as_quat()
    return quat

def quat_to_euler(qx=0, qy=0, qz=0, qw=0):
    '''
    Convert a quaternion to euler angles
    Assumes that the Euler angle rotation is EXTRINSIC and applied in the order of x, y, z
    Also assumes that the quaternion is given in scalar-last (x, y, z, w) format.
    The output angles are given in degrees (not rad)
    '''
    euler = Rotation.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)
    return euler

def robot_poses_as_htms(poses):
    '''
    The Kortex API gives robot poses in the format [x,y,z,theta_x,theta_y,theta_z]
    This function converts an array of poses (nx6) to an array of 4x4 homogeneous transformation matrices (nx4x4)
    '''

    if len(poses.shape) == 1:
        # if only one pose is given, reshape it to a 2D array with just one row
        poses = poses.reshape(1, -1)

    htms = np.zeros([len(poses), 4, 4])
    htms[:,:3,:3] = np.asarray([euler_to_mat(p[3], p[4], p[5]) for p in poses])
    htms[:,3,3] = 1
    htms[:,:3,3] = poses[:,:3]
    return htms
    
def invert_transform(matrix):
    """Inverts a 4x4 homogeneous transformation matrix."""
    R_inv = matrix[:3, :3].T  # Transpose of the rotation part
    t_inv = -R_inv @ matrix[:3, 3]  # Negative product of R_inv and translation part
    inverted_matrix = np.eye(4)
    inverted_matrix[:3, :3] = R_inv
    inverted_matrix[:3, 3] = t_inv
    return inverted_matrix