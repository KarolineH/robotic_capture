
from cam_io import EOS
import helpers.gphoto_util as gphoto_util
from time import time
from multiprocessing import Pool

cam_list = gphoto_util.detect_EOS_cameras()
nr_cams = len(cam_list)

def f(args):
    i = args[0]
    cam = EOS(port=cam_list[i])
    cam.sync_date_time()
    start = args[1]
    while time() < start + 5: #syncronize the execution of each process
        pass
    print(f'Cam {i} trigger down: {time()} \n')
    cam.record_video(10, False, '.')
    print(f'Cam {i} trigger up: {time()} \n')

if __name__ == "__main__":

    start = time()
    arguments = [(i, start) for i in range(nr_cams)]
    with Pool(nr_cams) as p:
        p.map(f, arguments)