import apriltag
import numpy as np

def create_detector(family="tag36h11", lower_requirements=False):
    if lower_requirements:
        options = apriltag.DetectorOptions(families=family, refine_edges=False, quad_contours=False)
        # these options refine_edges and quad_contours are set to False to circumvent a recurring error with the apriltag library:
        # "too many borders in contour_detect (max of 32767!)"
    else:
        options = apriltag.DetectorOptions(families=family)
    detector = apriltag.Detector(options)
    return detector

def create_pattern(tag_size=0.03, tag_spacing=0.015, tag_layout=np.array([4,5])):
    # All tags in the pattern are of the same tag family to distinguish them from other tags in the environment.
    # Measurements are provided in meters.
    # Layout is provided in [rows x columns]
    # old pattern: tag_size=0.0298235294118, tag_spacing=0.0149117647058, tag_layout=np.array([4,6]
    spacing = tag_spacing + tag_size
    corners_top_left = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)
    corners_top_right = corners_top_left + np.array([tag_size, 0, 0])
    corners_bottom_right = corners_top_left + np.array([tag_size, tag_size, 0])
    corners_bottom_left = corners_top_left + np.array([0, tag_size, 0])

    corner_array = np.stack((corners_top_left, corners_top_right, corners_bottom_right, corners_bottom_left), axis=1) # dimensions are (tags x 4 x 3)
    # clockwise order: top left, top right, bottom right, bottom left

    return corner_array