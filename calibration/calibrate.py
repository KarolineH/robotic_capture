import os
import cv2
import numpy as np

# Local imports
try:
    # Attempt a relative import
    from .helpers import april_tag_util # if being run as a package
    from .helpers import colmap_wrapper
    from .helpers import colmap_io
    from .helpers import opencv_conversions
    plotting = None # this is usually only used for debugging, so not needed when importing as a package
except ImportError:
    from helpers import april_tag_util, plotting, colmap_wrapper, colmap_io # local case
    from helpers import opencv_conversions

class CamCalibration:

    def __init__(self, cam_id, im_dir):
        self.cam_id = cam_id
        self.im_dir = im_dir
        self.resolution = None
        self.matrix = None
        self.distortion = None

    def april_tag_calibration(self, cam_mat=None, dist_coeff=None, pattern_in_world=np.eye(4), im_dir=None, lower_requirements=False, cam_model='OPENCV'):
        """
        OpenCV camera calibration using AprilTags as a calibration pattern.

        Calibrate the camera using images taken of a pre-made pattern of AprilTags. Can estimate intrinsics and extrinsics, or just extrinsics if intrinsics are known.
        Assumes that all images were taken with the same camera, at the same resolution and focal length, and have not been rotated.
        :param cam_mat: initial camera matrix, if known
        :param dist_coeff: initial distortion coefficients, if known
        :param pattern_in_world: 4x4 homogeneous transformation matrix describing the pose of the pattern in the world frame. If not provided, the pattern is assumed to be at the origin of the world frame.
        :param im_dir: optionally specify a different directory containing calibration images
        :param lower_requirements: set to True if the apriltag detection fails due to too many borders being detected in contours.
        :param cam_model: 'OPENCV' returns instrinsics [fx, fy, cx, cy, k1, k2, p1, p2, k3], 'FULL_OPENCV' returns [fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6]
        :return: dict of intrinics (incl. RMS re-projection error, camera matrix, distortion coefficients), estimated rotation vectors and translation vectors for all provided images
        """
        # cv2 (opencv) uses a right-handed coordinate system with positive axes going x-right, y-down, z-forward (away from camera through the image plane)
        # x=red, y=green, z=blue
        # We use the same convention throughout this function.

        flag_dict = {'OPENCV': 0, 'FULL_OPENCV': cv2.CALIB_RATIONAL_MODEL, 'SIMPLE_PINHOLE': cv2.CALIB_FIX_ASPECT_RATIO + cv2.CALIB_ZERO_TANGENT_DIST + cv2.CALIB_FIX_K1 + cv2.CALIB_FIX_K2 + cv2.CALIB_FIX_K3 + cv2.CALIB_FIX_K4 + cv2.CALIB_FIX_K5 + cv2.CALIB_FIX_K6}

        # Aspect ratio (optional, set to 1.0 for square pixels)
        if cam_model == 'SIMPLE_PINHOLE' and cam_mat is None:
            cam_mat = np.eye(3, dtype=np.float64) # to fix the aspect ratio to 1.0


        plot = False # useful for debugging, set to True to plot the axes of the first detected tag into each image
        if im_dir is None:
            im_dir = self.im_dir

        detector = april_tag_util.create_detector(lower_requirements=lower_requirements)
        corner_array = april_tag_util.create_pattern()

        obj_points = []
        img_points = []
        used_images = []
        resolution = None
        # Detect tags in all provided images
        for image in sorted(os.listdir(im_dir)):
            if not (image.endswith('.jpg') or image.endswith('.JPG')):
                continue
            img = cv2.imread(im_dir+'/'+image) # row, column
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            results = detector.detect(gray) # if only part of the pattern is visible, we still use all available tags
            # This line (above) sometimes causes a segmentation fault. This is not caught here.
            # If the warning "too many borders in contour_detect (max of 32767!)" is shown, try setting the input lower_requirements=True

            if len(results) > 5: # OPENCV requires at least 6 detected locations in an image for intrinsic + extrinsic camera calibration
                detected_ids = np.asarray([r.tag_id for r in results])
                valid_detected_ids = detected_ids[np.where(detected_ids <=corner_array.shape[0])] # only use detected tags that are part of the pattern
                image_coords = np.asarray([r.corners[0,:] for r in results], dtype=np.float32)
                valid_image_coords = image_coords[np.where(detected_ids <=corner_array.shape[0])]
                world_coords = corner_array[valid_detected_ids-1] # -1 because the first tag is tag 1 not tag 0
                obj_points.append(world_coords)
                img_points.append(valid_image_coords)
                used_images.append(image)
                if plot and plotting is not None:
                    plotting.plot_detected_april_corners(im_dir+'/'+image, valid_image_coords)

            if resolution is None:
                resolution = gray.shape[::-1] # width, height 

        # Calibrate the camera based on all detected tags in all provided images
        # if no initial camera matrix or distortion coefficients are provided, they are calibrated along with the extrinsics
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, resolution, cam_mat, dist_coeff, flags=flag_dict[cam_model]) # arguments are object points, image points, image size, camera matrix, distortion coefficients
        #rvecs, tvecs are the rotation and translation of the pattern origin given in the camera frame

        if plot and plotting is not None:
            # For debugging purposes, here the option to plot the pattern origin axes into each image. This is useful to check if the detections and conversions are correct.
            for image, rotation, translation in zip(used_images, rvecs, tvecs):
                plotting.plot_axes_on_img(im_dir+'/'+image, mtx, dist, rotation, translation) # plot the axes of the first detected tag into each image

        # the rotation and translation vectors bring the pattern from object frame to camera frame, that's the same as the pose of the pattern origin given in the camera frame
        homogeneous_transforms = [opencv_conversions.rodrigues_rvec_tvec_to_homogeneous(rvec, tvec) for rvec, tvec in zip(rvecs, tvecs)]
        obj_in_cam = np.array(homogeneous_transforms) # shape: (n_images, 4, 4), transformation matrices describing object location in the camera frame
        cam_in_obj = np.linalg.inv(obj_in_cam) # camera poses relative to the pattern origin
        cam_in_world = pattern_in_world @ cam_in_obj # camera poses relative to the world frame

        self.resolution = resolution
        self.matrix = mtx
        self.distortion = dist

        if plot and plotting is not None:
            plotting.plot_transforms(cam_in_world)

        return resolution, mtx, dist, cam_in_world, used_images # image resolution, camera matrix, distortion coefficients, 4x4 homogeneous transforms of the camera poses in world frame, also a list of which images were suitable for calibration and thus poses were estimated
    

    def colmap_calibration(self, im_dir=None, cam_model='OPENCV', force_rerun=False):
        """
        COLMAP camera calibration. Intended for calibration intrinsics and extrinsics together.

        Calibrate the camera using a provided series of calibration images.
        Assumes that all images were taken with the same camera, at the same resolution and focal length, and have not been auto-rotated.
        Currently assumes sequential image series and the OpenCV camera model.
        Does not take any known camera poses, so the reference frame of the reconstructed model is NOT anchored to the real world frame, but most likely the first image or a normalized center.
        Please note that this calibration method creates COLMAP files in the image directory.
        :param im_dir: optionally specify a different directory containing calibration images
        :param cam_model: 'OPENCV' returns instrinsics [fx, fy, cx, cy, k1, k2, p1, p2], 'FULL_OPENCV' returns [fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6]
        :return: ...
        """

        # Run COLMAP
        # Options:
            # OPENCV model yields fx, fy, cx, cy, k1, k2, p1, p2
            # FULL_OPENCV model yields fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
            # (SIMPLE_RADIAL yields f, cx, cy, k)
            # (SIMPLE_PINHOLE yields f, cx, cy)
            # exhaustive_matcher or sequential_matcher
            # If a sparse reconstruction model already exist in the folder, it does not run again. But you can force it to run again by setting force_rerun=True.
        if im_dir is None:
            im_dir = self.im_dir
        colmap_wrapper.gen_poses(basedir=im_dir, match_type='exhaustive_matcher', model_type=cam_model, force_rerun=force_rerun)

        # Retrieve the camera parameters from the colmap output
        camerasfile = os.path.join(self.im_dir, 'sparse/0/cameras.bin')
        camdata = colmap_io.read_cameras_binary(camerasfile)
        list_of_keys = list(camdata.keys())
        cam = camdata[list_of_keys[0]]
        resolution = np.array([cam.width, cam.height]) # width, height
        mtx = np.array([[cam.params[0],0,cam.params[2]],[0,cam.params[1],cam.params[3]],[0,0,1]])
        dist = np.array(cam.params[4:])

        # Retrieve the camera poses from the colmap output
        cam_in_world = colmap_io.load_camera_poses(self.im_dir) #c2w: camera to world transformation matrices for each frame
        # these assume a frame only specified as [r, -u, t], which we'll assume is x-right, y-down, z-through the image plane] (bc colmap uses right handed frames)

        # poses,_ = load_data(self.base_dir, load_imgs=False)
        # from the regular COLMAP output orientation, this is rotated so that the targeted frame is: x-down, y-right, and x-backwards/towards the camera.
        # Each poses[:,:-1,0] is a 3x4 homogeneous transformation matrix, with the last row left out (because it is always [0,0,0,1])
        # w2c_mats and c2w_mats are avaliable also in pose_utils
        #plotting.plot_transforms(cam_in_world)
        return resolution, mtx, dist, cam_in_world
    

if __name__ == '__main__':
    pass
    # Run the calibration routine
    cc = CamCalibration('EOS01','/home/karo/ws/data/calibration_images/intrinsics_calib/2024-05-16_16-41-12')#cam_calib/2024-05-16_16-41-12')
    #cc.colmap_calibration(cam_model='FULL_OPENCV', force_rerun=True)
    out = cc.april_tag_calibration(lower_requirements=True, cam_model='SIMPLE_PINHOLE')
    print(out)