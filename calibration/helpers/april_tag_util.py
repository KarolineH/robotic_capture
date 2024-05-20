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
    spacing = tag_spacing + tag_size
    corner_array = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)
    return corner_array