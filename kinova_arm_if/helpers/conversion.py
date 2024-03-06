from scipy.spatial.transform import Rotation

def euler_to_mat(theta_x=0, theta_y=0, theta_z=0):
    # Convert euler angles to rotation matrix
    # assumes that the rotation is extrinsic and applied in the order of x, y, z
    # also assumes that the angles are given in degrees
    matrix = Rotation.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_matrix()
    return matrix

def mat_to_euler(matrix):
    # Convert rotation matrix to euler angles
    # assumes that the rotation is extrinsic and applied in the order of x, y, z
    # also returns the angles in degrees
    euler = Rotation.from_matrix(matrix).as_euler('xyz', degrees=True)
    return euler

def euler_to_quat(theta_x=0, theta_y=0, theta_z=0):
    # Convert euler angles to quaternion
    # assumes that the rotation is extrinsic and applied in the order of x, y, z
    # also assumes that the angles are given in degrees
    quat = Rotation.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_quat()
    return quat