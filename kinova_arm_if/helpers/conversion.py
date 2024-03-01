from scipy.spatial.transform import Rotation

def euler_to_mat(theta_x, theta_y, theta_z):
    # Convert euler angles to rotation matrix
    # assumes that the rotation is extrinsic and applied in the order of x, y, z
    # also assumes that the angles are given in degrees
    matrix = Rotation.from_euler('xyz', [theta_x, theta_y, theta_z], degrees=True).as_matrix()
    return matrix