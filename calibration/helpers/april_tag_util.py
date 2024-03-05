import apriltag
import numpy as np
import cv2


def create_detector(family="tag36h11"):
    options = apriltag.DetectorOptions(families=family)
    detector = apriltag.Detector(options)
    return detector

def create_pattern(tag_size=0.0298235294118, tag_spacing=0.0149117647058, tag_layout=np.array([4,6])):
    # All tags in the pattern are of the same tag family to distinguish them from other tags in the environment.
    # Measurements are provided in meters.
    # Layout is provided in [rows x columns]
    spacing = tag_spacing + tag_size
    corner_array = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)
    return corner_array

def plot_marker_axes(img_file, cam_mtx, dist_coeff, rotation, translation):
    
    img = cv2.imread(img_file) # row, column
    im2 = cv2.drawFrameAxes(img, cam_mtx, dist_coeff, rotation, translation, 0.1, 6)

    # rescale the image to fit on screen
    scale_percent = 15 # percent of original size
    width = int(im2.shape[1] * scale_percent / 100)
    height = int(im2.shape[0] * scale_percent / 100)
    dim = (width, height)
    im2 = cv2.resize(im2, dim, interpolation = cv2.INTER_AREA)

    cv2.imshow("Image", im2)
    cv2.waitKey(0)
    return