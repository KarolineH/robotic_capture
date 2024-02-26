import matplotlib.pyplot as plt
import cv2
from pytransform3d.transform_manager import TransformManager

def plot_transforms(transforms):
    tm = TransformManager()
    if len(transforms.shape) == 3:
        print('Transforms shape:', transforms.shape)
        num_poses = transforms.shape[0]
        for i in range(num_poses):
            pose = transforms[i, :, :]
            tm.add_transform(f'Pose {i + 1}','world', pose)
    elif len(transforms.shape) == 2:
        print('Transforms shape:', transforms.shape)
        tm.add_transform(f'Pose {i + 1}','world', transforms)
        
    ax = tm.plot_frames_in("world", s=0.1)
    ax.set_xlim((-0.25, 0.75))
    ax.set_ylim((-0.5, 0.5))
    ax.set_zlim((0.0, 1.0))
    plt.show()
    return

def plot_axes_on_img(img_file, cam_mtx, dist_coeff, rotation, translation):
    
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
    cv2.destroyAllWindows()
    return