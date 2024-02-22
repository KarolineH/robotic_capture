import websocket
import subprocess as sp
import numpy as np
import time

class ASTORA_Lights(object):
    """
    Interface one or multiple ASTORA Soft Panels via DMX.
    """

    def __init__(self, num_lights=2, num_knobs=2):
        self.num_lights = num_lights
        self.num_knobs = num_knobs
        self.channels = num_lights * num_knobs
        self.port = 9999        # Param: I don't know if qlc+ can use a different port
        self.fade_frequency = 1 # Hz, use this as an upper bound only, the actual possible highest frequency is determined by the websocket connection. Must be > 0.

        self.init_connection()
        return

    def init_connection(self):
        """
        Launch the QLC+ server and connect to it via websocket.
        The server always launches with GUI, so make sure not to close that window.
        The connected lights will be detected automatically.
        """
        self.server = sp.Popen(['qlcplus', '--web', '--nowm', '--nogui'], stdout=sp.DEVNULL) # nogui option not working currently
        self.ws = websocket.WebSocket()
        time.sleep(5)
        self.ws.connect(f'ws://127.0.0.1:{self.port}/qlcplusWS')
        return

    def get_light_parameters(self):
        """
        Get the current intensity and colour temperature of all lights.
        Output: arrays intensity(1xn) and colour_temp(1xn) for n = number of lights`
        """

        # Command syntax: 'api trigger phrase|command|universe|channel starting index|channel range'
        self.ws.send(f'QLC+API|getChannelsValues|1|1|{self.channels}')
        out = self.ws.recv()
        channel_values = np.asarray(out.split('|')[2:]).reshape(-1,3)[:,:2].astype(int) # reformat into the individual channels
        knob_values = channel_values[:,1].reshape(self.num_knobs,self.num_lights) # reshape into the individual knobs, shape (num_knobs, num_lights)
        intensity = knob_values[0,:].tolist()
        colour_temp = knob_values[1,:].tolist()
        return intensity, colour_temp

    def send_params(self, params):
        for channel, value in enumerate(params):
            self.ws.send(f'CH|{channel + 1}|{value}')
        return
        
    def set_light_parameters(self, intensity, colour_temp, duration=None):
        """
        Set the intensity and colour temperature for all connected lights.
        Input: arrays intensity(1xn) and colour_temp(1xn) for n = number of lights, accepts values between 0 and 255
        Optional: duration in seconds, if provided the lights will fade to the new values over the specified duration
        Output: (new))active intensity and colour temperature, and a status message
        """
        # Check input validity
        msg = ''
        if len(intensity) < 1 or len(colour_temp) < 1 or len(intensity) != len(colour_temp):
            msg = 'Invalid number of parameters. Specify one intensity and one colour temperature per light.'
            print(msg)
            return None, None, msg
        
        # Reshape and clip if necessary
        target_array = np.column_stack((intensity, colour_temp)).reshape(-1)
        if not all([0 <= value <= 255 for value in target_array]):
            target_array = target_array.clip(0,255)
            msg += f'Some Parameters were limited to the accepted interval 0-255. '


        # If a duration is provided, fade the lights
        # The smoothness of the fade is limited by the frequency of the websocket connection
        # The exact duration also sometimes varies by a small margin, due to comms time
        # TODO: Determine the intermediate step parameters by time passed in each loop instead of pre-calculating the needed steps and sleeping for a fixed time
        if duration is not None and duration > 0 and self.fade_frequency > 0:
            start_array = np.column_stack(self.get_light_parameters()).reshape(-1)
            if not start_array.shape == target_array.shape:
                msg += 'Please specify one intensity and one colour temperature for EACH light, the array shape must match the current set-up.'
                print(msg)
                return None, None, msg

            num_steps = int(duration * self.fade_frequency)
            time_per_step = 1 / self.fade_frequency
            intermediate_vals = np.zeros((num_steps,) + start_array.shape, dtype=int)
            
            # Calculate intermediate values
            for step in range(num_steps):
                t = (step+1) * time_per_step / duration
                intermediate_vals[step] = np.rint(start_array + (target_array - start_array) * t).astype(int)
            # Execute the fade in loop
            for step in range(num_steps):
                self.send_params(intermediate_vals[step])
                time.sleep(time_per_step)
            msg += f'{duration} second light fade completed.'
        
        else:
            # Send the commands via websocket
            self.send_params(target_array)
            msg += f'{len(target_array)} Light parameters set.'

        # Return the current values and a status message
        current_intensity, current_colour_temp = self.get_light_parameters()
        return current_intensity, current_colour_temp, msg
    
    def disconnect(self):
        self.ws.close()
        self.server.terminate()
        return

if __name__ == '__main__':
    lights = ASTORA_Lights()

    # Usage examples
    print(lights.get_light_parameters())
    lights.set_light_parameters([155,200],[211,256]) # set immediately
    lights.set_light_parameters([0,0],[0,0], duration=10) # fade over 10 seconds
    lights.disconnect()
