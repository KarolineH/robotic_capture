
import cv2
import time
import threading

# Local imports
try:
    from .helpers import rec_util # if being run as a package
except ImportError:
    import helpers.rec_util as rec_util

class Recorder:
    def __init__(self, device_name=None):
        
        # Select the video capture device
        self.device = device_name
        if device_name is None:
            self.device = rec_util.choose_camera()
        # Open the video capture device
        self.cap = cv2.VideoCapture(device_name)
        if not self.cap.isOpened():
            print("Cannot open camera")
            exit()
        self.resolution = self.set_resolution(1920, 1080)

        # Initialise recording variables
        self.out = None
        self.recording_thread = None
        self.is_recording = False
        # Output is BGR24 by default, for more info and to change this, look up cv2.CAP_PROP_MODE (==0).
        #self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)


    def set_resolution(self, width, height):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return (width, height)
    
    def _write_frame(self):
        while self.is_recording and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            # Write the frame to the output file
            self.out.write(frame)

    def start_recording(self, filepath):
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.out = cv2.VideoWriter(filepath, fourcc, fps, self.resolution)

        self.is_recording = True
        self.recording_thread = threading.Thread(target=self._write_frame)
        self.recording_thread.start()

    def stop_recording(self):
        self.is_recording = False
        if self.recording_thread is not None:
            self.recording_thread.join()  # Wait for the recording thread to finish

        if self.cap is not None and self.out is not None:
            self.cap.release()
            self.out.release()

    def show_live(self):
        '''
        Display a live video feed until 'q' is pressed.
        '''
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            if not ret:
                print("Can't receive frame. Exiting ...")
                break

            # Display the resulting frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) == ord('q'):
                break

    def record(self, output_file, duration=None):
        '''
        OBSOLTETE
        Record a video to a sepcified file.
        TODO: Implement a way to stop the recording after a certain duration.
        Optionally, specify a duration (in seconds) to record for.
        Otherwise interrupt the recording with Ctrl+C when done.
        '''

        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_file, fourcc, 60, (1920, 1080))
        start = time.time()

        try:
            if duration is None:
                print("Recording. Press Ctrl+C to stop.")
                while True:
                    ret, frame = self.cap.read()
                    if not ret:
                        print("Error: Failed to capture frame")
                        break
                    # Write the frame to the output file
                    out.write(frame)
            else:
                print(f"Recording for {duration} second(s).")
                while time.time() - start < duration:
                    ret, frame = self.cap.read()
                    if not ret:
                        print("Error: Failed to capture frame")
                        break
                    # Write the frame to the output file
                    out.write(frame)
        except KeyboardInterrupt:
            out.release()

    def close(self):
        self.cap.release()


if __name__ == '__main__':
    # instantiate a Recorder object
    device_name = rec_util.choose_camera()
    rec = Recorder(device_name)
    # record to a file for 3 seconds
    rec.start_recording('output.mp4')
    time.sleep(3)
    rec.stop_recording()
    # show a live video feed, interrupt by pressing 'q'
    rec.show_live()
    # close the video capture device
    rec.close()
