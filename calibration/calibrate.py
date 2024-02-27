import os
import cv2
import numpy as np
from helpers import april_tag_util, io_util, transform_util, plotting

class CamCalibration:

    def __init__(self, name, im_dir):
        self.name = name
        self.im_dir = im_dir

    def april_tag_calibration(self, cam_mat=None, dist_coeff=None, pattern_in_world=np.eye(4), im_dir=None):
        """
        OpenCV camera calibration using AprilTags as a calibration pattern.

        Calibrate the camera using images taken of a pre-made pattern of AprilTags. Can estimate intrinsics and extrinsics, or just extrinsics if intrinsics are known.
        Assumes that all images were taken with the same camera, at the same resolution and focal length, and have not been rotated.
        :param cam_mat: initial camera matrix, if known
        :param dist_coeff: initial distortion coefficients, if known
        :param pattern_in_world: 4x4 homogeneous transformation matrix describing the pose of the pattern in the world frame. If not provided, the pattern is assumed to be at the origin of the world frame.
        :param im_dir: optionally specify a different directory containing calibration images
        :return: dict of intrinics (incl. RMS re-projection error, camera matrix, distortion coefficients), estimated rotation vectors and translation vectors for all provided images
        """
        # cv2 (opencv) uses a right-handed coordinate system with positive axes going x-right, y-down, z-forward (away from camera through the image plane)
        # x=red, y=green, z=blue
        # We use the same convention throughout this function.

        plot = False # useful for debugging, set to True to plot the axes of the first detected tag into each image
        if im_dir is None:
            im_dir = self.im_dir

        detector = april_tag_util.create_detector()
        corner_array = april_tag_util.create_pattern()

        obj_points = []
        img_points = []
        # Detect tags in all provided images
        for image in os.listdir(im_dir):
            img = cv2.imread(im_dir + image) # row, column
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray) # if only part of the pattern is visible, we still use all available tags
            detected_ids = np.asarray([r.tag_id for r in results])
            image_coords = np.asarray([r.corners[0,:] for r in results], dtype=np.float32)
            world_coords = corner_array[detected_ids]

            obj_points.append(world_coords)
            img_points.append(image_coords)
        resolution = gray.shape[::-1] # width, height 

        # Calibrate the camera based on all detected tags in all provided images
        # if no initial camera matrix or distortion coefficients are provided, they are calibrated along with the extrinsics
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], cam_mat, dist_coeff) # arguments are object points, image points, image size, camera matrix, distortion coefficients

        if plot: 
            for image, rotation, translation in zip(os.listdir(im_dir), rvecs, tvecs):
                plotting.plot_axes_on_img(im_dir + image, mtx, dist, rotation, translation) # plot the axes of the first detected tag into each image

        # the rotation and translation vectors bring the pattern from object frame to camera frame, that's the same as the pose of the pattern origin given in the camera frame
        homogeneous_transforms = [transform_util.rvec_tvec_to_homogeneous(rvec, tvec) for rvec, tvec in zip(rvecs, tvecs)]
        obj_in_cam = np.array(homogeneous_transforms) # shape: (n_images, 4, 4), transformation matrices describing object location in the camera frame
        cam_in_obj = np.linalg.inv(obj_in_cam) # camera poses relative to the pattern origin
        cam_in_world = np.matmul(cam_in_obj, pattern_in_world) # camera poses relative to the world frame

        return resolution, mtx, dist, cam_in_world # image resolution, camera matrix, distortion coefficients, 4x4 homogeneous transforms of the camera poses in world frame

if __name__ == "__main__":
    #im_dir = '/home/kh790/data/april_tag_imgs/'
    im_dir = '/home/karo/ws/data/calibration_images/april_tag/'
    cam_cal = CamCalibration('test_cam', im_dir)
    #res,mtx,dist,transforms = cam_cal.april_tag_calibration()
    #plotting.plot_transforms(transforms)
    #io_util.save_to_yaml('calibration.yaml', cam_cal.name, res, mtx, dist)
    name, size, k, d = io_util.load_from_yaml('calibration.yaml')
    print(name, size, k, d)