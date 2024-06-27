from cam_io import EOS
import helpers.gphoto_util as gphoto_util
import time
from multiprocessing import Pool

''' 
This module provides a class SyncRecorder that allows to record videos with multiple cameras in synchronisation.
The class provides the following methods:
- start_all(): Start recording a video without fixed duration with all connected cameras in synchronisation.
- stop_all(): Stop recording video for all connected cameras in synchronisation.
- rec_all_for_duration(duration): Record a video of a given duration with all connected cameras in synchronisation.

Note: communication with the cameras has some inherent delays. When using the start and stop methods, 
the actual duration of the recorded video can be slightly longer than the waiting time between function calls (~within 2 seconds).

Note: A common print-out "No matching processes belonging to you were found" can be safely ignored.

Note: Make sure to set all your desired capture parameters in the Camera menu directly, as this script is only intended for synchronisation.
'''


class SyncRecorder:
    def __init__(self):
        self.delay = 1 # Fixed delay(in seconds) before executing any command, to ensure synchronisation
        self.cam_list = gphoto_util.detect_EOS_cameras()
        self.nr_cams = len(self.cam_list)

    def start(self, args):
        ''' 
        Starts recording a video without fixed duration (single process)
        '''
        i = args[0] # Camera index
        t0 = args[1] # Time at which the command was given, which is the same for all cameras and is used to synchronise the execution of each process

        cam = EOS(port=self.cam_list[i]) # Connect to the camera
        cam.sync_date_time() # synchronise the camera time with the system time

        while time.time() < t0 + self.delay: #synchronise the execution of each process
            pass
        cam.start_rec_vid() # START THE RECORDING

    def stop(self, args):
        '''
        Stops recording video (single process)
        '''
        i = args[0] # Camera index
        t0 = args[1] #  Time at which the command was given, which is the same for all cameras and is used to synchronise the execution of each process

        cam = EOS(port=self.cam_list[i]) # Connect to the camera

        while time.time() < t0 + self.delay: #synchronise the execution of each process
            pass
        __,__,msg = cam.stop_rec_vid(download=False) # STOP THE RECORDING
        return msg

    def rec_for_duration(self, args):
        ''' 
        Records a video of a given duration (single process)
        '''
        i = args[0] # Camera index
        t0 = args[1] #  Time at which the command was given, which is the same for all cameras and is used to synchronise the execution of each process
        duration = args[2] # Duration of the video

        cam = EOS(port=self.cam_list[i]) # Connect to the camera
        cam.sync_date_time() # synchronise the camera time with the system time

        while time.time() < t0 + self.delay: #synchronise the execution of each process
            pass
        # print(f'Cam {i} trigger down: {time()} \n') # for debugging
        cam.record_video(duration, False, '.') # RECORD VIDEO
        # print(f'Cam {i} trigger up: {time()} \n') # for debugging

    def start_all(self):
        ''' 
        Starts recording a video without fixed duration with all connected cameras in synchronisation.
        '''
        t0 = time.time()
        arguments = [(i, t0) for i in range(self.nr_cams)]
        with Pool(self.nr_cams) as p:
            p.map(self.start, arguments)

    def stop_all(self):
        ''' 
        Stops recording video for all connected cameras in synchronisation.
        '''
        t0 = time.time()
        arguments = [(i, t0) for i in range(self.nr_cams)]
        with Pool(self.nr_cams) as p:
            msgs = p.map(self.stop, arguments)
        return msgs
    
    def rec_all_for_duration(self, duration):
        ''' 
        Records a video of a given duration with all connected cameras in synchronisation.
        '''
        t0 = time.time()
        arguments = [(i, t0, duration) for i in range(self.nr_cams)]
        with Pool(self.nr_cams) as p:
            p.map(self.rec_for_duration, arguments)


if __name__ == "__main__":
    rec = SyncRecorder()
    rec.start_all()
    time.sleep(6)
    rec.stop_all()
    #rec.rec_all_for_duration(5)