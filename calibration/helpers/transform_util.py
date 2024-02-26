import cv2
import numpy as np

# A function to convert rotation and translation vectors to homogeneous transformation matrices
def rvec_tvec_to_homogeneous(rvec, tvec):
    """
    Convert rotation and translation vectors to homogeneous transformation matrix
    :param rvec: rotation vector
    :param tvec: translation vector
    :return: 4x4 homogeneous transformation matrix
    """
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    homogeneous_transform = np.eye(4)
    homogeneous_transform[:3, :3] = rotation_matrix
    homogeneous_transform[:3, 3] = tvec.flatten()
    return homogeneous_transform