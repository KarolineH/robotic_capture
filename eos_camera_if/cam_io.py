import gphoto2 as gp
import subprocess as sp, logging, os
import time
from subprocess import Popen, PIPE

class EOS(object):
    """
    Interface a Canon EOS R5 C using gphoto2 via USB port.

    Quickstart: Take a look first at the top-level API calls: get_capture_parameters(), capture_image(), capture_video(), and show_live_preview().
    """

    def __init__(self, port=None):
        # Kill any existing gphoto processes to free up the USB ports for communication
        # prevents error *Could not claim the USB device*
        command = f'killall gvfsd-gphoto2 gvfs-gphoto2-volume-monitor'
        sp.call([command], shell=True)

        camera_list = list(gp.Camera.autodetect()) # Find all available cameras
        if not camera_list:
            print('No camera detected')
            exit()
        
        self.camera = gp.Camera()
        if port is not None: # If a port is specified, initialise the correct device, otherwise just use the first detected compatible device
            port_info_list = gp.PortInfoList()
            port_info_list.load()
            idx = port_info_list.lookup_path(port)
            self.camera.set_port_info(port_info_list[idx])

            name = camera_list[[entry[1] for entry in camera_list].index(port)][0]
            abilities_list = gp.CameraAbilitiesList()
            abilities_list.load()
            idx = abilities_list.lookup_model(name)
            self.camera.set_abilities(abilities_list[idx])

        # Initialise camera
        self.camera.init()
        self.config = self.camera.get_config()
        self.mode = self.get_camera_mode() # detects the manual switch state: 0 == PHOTO, 1 == VIDEO
        self.check_storage_medium() # check if an SD card is inserted and warn the user if not
        if self.mode == 0:
            self.set_exposure_manual() # set the camera's auto-exposure mode to manual, so that shutter, aperture, and iso can be set remotely
            self.set_save_target() # set the camera's save target to the SD card, so that all captures are saved to the SD card by default
            self.set_capture_parameters(aperture='AUTO', iso='AUTO', shutterspeed='AUTO', c_AF=False) # set the camera to automatic

        # set the main capture configuration options for both PHOTO and VIDEO mode
        # These ares specific to the Canon EOS R5 C
        if self.mode == 0:
            self.aperture_choices = [2.8, 3.2, 3.5, 4, 4.5, 5, 5.6, 6.3, 7.1, 8, 9, 10, 11, 13, 14, 16, 18, 20, 22, 25, 29, 32]
            self.shutter_choices = ['30', '25', '20', '15', '13', '10.3', '8', '6.3', '5', '4', '3.2', '2.5', '2', '1.6', '1.3', '1', '0.8', '0.6', '0.5', '0.4', '0.3', '1/4', '1/5', '1/6', '1/8', '1/10', '1/13', '1/15', '1/20', '1/25', '1/30', '1/40', '1/50', '1/60', '1/80', '1/100', '1/125', '1/160', '1/200', '1/250', '1/320', '1/400', '1/500', '1/640', '1/800', '1/1000', '1/1250', '1/1600', '1/2000', '1/2500', '1/3200', '1/4000', '1/5000', '1/6400', '1/8000']
            self.iso_choices = [100, 125, 160, 200, 250, 320, 400, 500, 640, 800, 1000, 1250, 1600, 2000, 2500, 3200, 4000, 5000, 6400, 8000, 10000, 12800, 16000, 20000, 25600, 32000, 40000, 51200]
        else:
            self.aperture_choices = [2.8, 3.2, 3.5, 4, 4.5, 5, 5.6, 6.3, 7.1, 8, 9, 10, 11, 14, 16, 18, 20, 22, 25, 29, 32] # option 13 is missing
            self.shutter_choices = ['1/50', '1/60', '1/75', '1/90', '1/100', '1/120', '1/150', '1/180','1/210', '1/250', '1/300', '1/360',  '1/420',  '1/500',  '1/600',  '1/720',  '1/840',  '1/1000', '1/1200', '1/1400', '1/1700', '1/2000']
        
        time.sleep(1) # wait for the camera to initialise


    ''' Universal Methods, work in both PHOTO and VIDEO mode '''

    def set_config_and_confirm(self, config_names, values, timeout=6):
        '''
        Helper function to set and 'push' a list of new configurations to the camera.
        This function then also fetches the currently active configuration from the camera to confirm that the named configurations have been updated successfully.
        '''

        # TODO: add timeout and warning

        # First, change all the given values
        try:
            for config_name, value in zip(config_names, values):
                conf = gp.check_result(gp.gp_widget_get_child_by_name(self.config, config_name))
                conf.set_value(value)
        except Exception as err:
            print(f"Unhandled gphoto2 error: ({err}) while setting config {config_name} to {value}")
            return False

        # Then push all changes to the camera
        success = False
        while not success:
            try:
                OK = gp.check_result(gp.gp_camera_set_config(self.camera, self.config))
                start = time.time()
                while not success and time.time() - start < timeout:
                    # Check if the camera has updated the configuration
                    # This should prevent any commands being skipped
                    new_config = self.camera.get_config()
                    for config_name, value in zip(config_names, values):
                        conf = gp.check_result(gp.gp_widget_get_child_by_name(new_config, config_name))
                        if conf.get_value() != value:
                            break
                    else:
                        success = True # this is only reached if the for loop is not broken 
                        self.config = new_config
                        break
                else:
                    print(f"Camera did not confirm new configuration within {timeout} seconds")
                    return False
            except Exception as err:
                if '-110' in str(err):
                    error_msg = f"Camera is busy, retrying..."
                else:
                    error_msg = f"Unhandled gphoto2 error: ({err}) while setting config {config_name} to {value}"
                # this is only here to catch an "I/O Busy" error and make sure the command is sent, even if the port is busy for a moment
                pass
        return success
    
    def set_config_fire_and_forget(self, config_name, value):
        '''
        Fast & unreliable, but essential for trigger-only settings, that need to simply overwrite the current value without waiting for a confirmation.

        Helper function to 'push' a new configuration to the camera.
        This function does not wait for a camera event, indicating that the named configuration has been updated.
        This function is faster, but trusts that the command was executed.
        '''
        success = False
        while not success:
            try:
                conf = gp.check_result(gp.gp_widget_get_child_by_name(self.config, config_name))
                conf.set_value(value)
                OK = gp.check_result(gp.gp_camera_set_config(self.camera, self.config))
                success = True
            except Exception as err:
                if '-110' in str(err):  # this is only here to catch an "I/O Busy" error and make sure the command is sent, even if the port is busy for a moment
                    error_msg = f"Camera is busy, retrying..."
                else:
                    error_msg = f"Unhandled gphoto2 error: ({err}) while setting config {config_name} to {value}"
                pass
        return success
    
    def list_all_config(self):
        '''
        List all available configuration options communicated via USB and supported by gphoto2, including those not (yet) implemented in this class.
        Output: List of strings
        '''
        return [el[0] for el in gp.check_result(gp.gp_camera_list_config(self.camera))]
    
    def get_camera_mode(self):
        '''
        Detect whether the physical switch on the camera is set to photo or video mode
        Output: int 0 == PHOTO, 1 == VIDEO
        '''
        switch = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'eosmovieswitch'))
        value = gp.check_result(gp.gp_widget_get_value(switch))
        return int(value)
    
    def sync_date_time(self):
        '''
        Sync the camera's date and time with the connected computer's date and time.
        '''
        self.set_config_fire_and_forget('syncdatetimeutc', 1)
        self.set_config_fire_and_forget('syncdatetimeutc', 0)
        return

    def get_config(self, config_name=None):
        '''
        Get the current value and all choices of a named configuration, including those not specifically implemented in this class (yet).
        Input: string, name of the configuration
        Output: tuple (string: current value, list of strings: choices)
        '''
        if type(config_name)==str:
            config_name = config_name.lower()
            if config_name in self.list_all_config():
                conf = gp.check_result(gp.gp_widget_get_child_by_name(self.config, config_name))
                value = gp.check_result(gp.gp_widget_get_value(conf))
                try:
                    choices = list(conf.get_choices())
                except:
                    choices = None
                    print(f"Config {config_name} provides no choices")
                return value, choices
            else:
                print(f"Config {config_name} not found")
                return None, None
        else:
            print(f"Config name must be a string")
            return None, None
        
    def check_storage_medium(self):
        '''
        Check if a supported SD card is inserted and warn the user if not.
        '''
        if len(list(self.camera.folder_list_folders('/'))) < 1:
            print('No storage medium detected')
            import warnings
            warnings.warn("Warning: No storage medium detected. Your captures might not be saved and you might run into errors later! Please make sure you have an SD card inserted and try again.")
            return False
        else:
            return True
        
    def get_file_info(self, file_path):
        '''
        Retrieve information about a specific file saved on the camera storage medium.
        Output: info object with variables info.file.size, info.file.type, info.file.mtime and more
        '''
        if type(file_path)==str:
            if len(file_path) > 0 and file_path[0] == '/' and file_path[-1] != '/':
                folder, name = os.path.split(file_path)
                try:
                    info = self.camera.file_get_info(folder, name)
                        # usage examples:
                        #size = info.file.size
                        #file_type = info.file.type
                        #timestamp = datetime.fromtimestamp(info.file.mtime).isoformat(' ')
                    return info
                
                except Exception as err:
                    if '-108' in str(err):
                        print(f"File {file_path} not found")
                    else:
                        print('Unhandled gphoto2 error: ' + err)
                    return None

            else:
                print(f"Please provide the absolute file path. Path {file_path} must be a string starting with '/' and ending with the file name")
                return None

    def list_files(self, path='/store_00020001/DCIM'):
        '''
        List all media files saved in the main media directory of the camera storage medium (default) or at another specified directory.
        Output: List of file paths (strings) in the given directory's immediate subdirectories. 
        '''
        if type(path)==str:
            if len(path) > 0 and path[0] == '/':
                dirs = [os.path.join(path, folder[0]) for folder in self.camera.folder_list_folders(path)]
                files = [os.path.join(folder,file_name[0]) for folder in dirs for file_name in self.camera.folder_list_files(folder)]
            else:
                print(f"Please provide the absolute path. Path {path} must be a string starting with '/'")
                return None
        else:
            print(f"Path must be a string")
            return None
        return files
    
    def download_file(self, camera_path, target_file=None):
        '''Download a specific file from the camera storage medium to the target file path on the PC.'''

        if type(camera_path)==str:
            if len(camera_path) > 0 and camera_path[0] == '/' and camera_path[-1] != '/':
                folder, name = os.path.split(camera_path)
                try:
                    cam_file = self.camera.file_get(folder, name, gp.GP_FILE_TYPE_NORMAL)
                except Exception as err:
                    if '-108' in str(err):
                        print(f"File {camera_path} not found")
                    else:
                        print('Unhandled gphoto2 error: ' + err)
                    return None
                if target_file is None:
                    target_file = os.path.join('./', name)
                cam_file.save(target_file)
                return target_file
            else:
                print(f"Please provide the absolute file path. Path {camera_path} must be a string starting with '/' and ending with the file name")
                return None
        else:
            print(f"Camera path must be a string")
            return None
    
    def manual_focus(self, value=3):
        '''
        Manually drive the lens focus nearer or further in increments of three different sizes.
        This function will have to be called repeatedly to achieve a specific focus distance.
        To bring the focus point nearer, use [0,1,2] for [small, medium, large] increments.
        To bring the focus point further, use [4,5,6] for [small, medium, large] increments.
        Note that the camera does NOT report an avilable range or when the maximum or minimum focus distance has been reached.
        Input: int 0-6
        Output: string describing the action taken
        '''
        choices = ['Near 1', 'Near 2', 'Near 3', 'None', 'Far 1', 'Far 2', 'Far 3']
        # 0,1,2 == small, medium, large increment --> nearer
        # 3 == none
        # 4,5,6 == small, medium, large increment --> further 
        value = int(value)
        if 0 <= value <= 2:
            msg = 'Manually focussing nearer'
        elif 4 <= value <= 6:
            msg = 'Manually focussing farther'
        elif value == 3:
            msg = 'Manual focus drive set to neutral'
        else:
            msg = f'Manual focus drive failed, value {value} out of range'
            return msg
        
        self.set_config_fire_and_forget('manualfocusdrive', choices[value])
        self.set_config_fire_and_forget('manualfocusdrive', 'None') # reset to neutral
        return msg
    
    def get_capture_parameters(self):
        '''Get the current values for aperture, iso, shutter speed, and continuous auto focus.'''
        aperture = self.get_aperture()
        shutterspeed = self.get_shutterspeed()
        c_AF = self.get_continuous_AF()
        iso = self.get_iso()
        return aperture, iso, shutterspeed, c_AF
    
    def get_aperture(self):
        '''Get the current aperture (f-number) setting.'''
        self.config = self.camera.get_config()
        aperture = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'aperture'))
        current = 'AUTO' if aperture.get_value() == 'Unknown value 00ff' or aperture.get_value() == 'implicit auto' else aperture.get_value()
        return current
    
    def get_shutterspeed(self):
        '''Get the current shutter speed setting.'''
        self.config = self.camera.get_config()
        shutterspeed = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'shutterspeed'))
        current = 'AUTO' if shutterspeed.get_value() == 'bulb' or shutterspeed.get_value() == 'auto' else shutterspeed.get_value()
        return current
    
    def get_continuous_AF(self):
        '''Get the current continuous auto focus setting.'''
        self.config = self.camera.get_config()
        if self.mode == 0:
            config = 'continuousaf'
        else:
            config = 'movieservoaf'
        c_AF = gp.check_result(gp.gp_widget_get_child_by_name(self.config, config))
        return c_AF.get_value()
    
    def get_iso(self):
        '''Get the current ISO setting.'''
        self.config = self.camera.get_config()
        if self.mode == 1:
            #TODO: Double check if there is no way to get this value in VIDEO mode
            return None
        iso = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'iso'))
        current = 'AUTO' if iso.get_value() == 'Auto' else iso.get_value()
        return current

    def set_capture_parameters(self, aperture=None, iso=None, shutterspeed=None, c_AF=None):
        '''Set the aperture, iso, shutter speed, and continuous auto focus.'''
        msgs = ''
        configs = []
        values = []

        ap_val, ap_msg = self.pick_aperture_value(aperture)
        iso_val, iso_msg = self.pick_iso_value(iso)
        ss_val, ss_msg = self.pick_shutterspeed_value(shutterspeed)
        cAF_val, cAF_config, cAF_msg = self.pick_continuous_AF_value(c_AF)

        msgs += ap_msg + iso_msg + ss_msg + cAF_msg

        for val, config in zip([ap_val, iso_val, ss_val, cAF_val], ['aperture', 'iso', 'shutterspeed', cAF_config]):
            if val is not None:
                configs.append(config)
                values.append(val)

        #self.set_aperture(ap_val)
        success = self.set_config_and_confirm(configs, values)
        msgs += '... Capture parameters set. '
        return msgs
    
    def set_aperture(self, value='AUTO'):
        '''
        Use this if you want to change ONLY the aperture (f-number).
        Always returns the (new) currently active setting and potential error messages.
        '''
        corrected_value, msg = self.pick_aperture_value(value)
        if corrected_value is None:
            return self.get_aperture(), msg
        #self.set_config_fire_and_forget('aperture', corrected_value)
        self.set_config_and_confirm(['aperture'], [corrected_value])
        current = self.get_aperture()
        return current, msg
        
    def set_shutterspeed(self, value='AUTO'):
        '''
        Use this if you want to change ONLY the shutter speed/ exposure time.
        Always returns the (new) currently active setting and potential error messages.
        '''
        corrected_value, msg = self.pick_shutterspeed_value(value)
        if corrected_value is None:
            return self.get_shutterspeed(), msg
        self.set_config_and_confirm(['shutterspeed'], [corrected_value])
        current = self.get_shutterspeed()
        return current, msg

    def set_continuous_AF(self, value='Off'):
        '''
        Use this if you want to change ONLY the continuous Auto-focus functionality.
        Always returns the (new) currently active setting and potential error messages.
        '''
        corrected_value, config, msg = self.pick_continuous_AF_value(value)
        if value is None:
            return self.get_continuous_AF(), msg
        self.set_config_and_confirm([config], [corrected_value])
        current = self.get_continuous_AF()
        return current, msg

    def pick_aperture_value(self, value='AUTO'):
        '''
        Helper function to check and format the input value used to change the aperture (f-number).
        The camera accepts slightly different inputs in PHOTO and VIDEO mode, so both are unified in this method.
        Input: int, float, numeric string, or the string 'AUTO'
        Output: the value (string) accepted by the camera, and a potential error message (string)

        WARNING: !! In VIDEO mode, AUTO setting is still untested. Might have to set 'Iris Mode' to 'Automatic' in the camera menu if you need auto aperture. !!
        '''
        if value is None:
            return None, 'Input is None, aperture unchanged. '

        msg = ''
        if value == 'AUTO':
            if self.mode == 0:
                value = 'Unknown value 00ff' # in PHOTO mode
            else:
                value = 'implicit auto' # in VIDEO mode
        else:
            try:
                value = float(value)
            except ValueError:
                msg = f"Aperture value {value} not supported. Please use string 'AUTO' or a number (int/float/numeric string)."
                print(msg)
                print('Supported numeric values: ', self.aperture_choices)
                return None, msg

            # if the exact value specified is not supported, use the closest option
            if value not in self.aperture_choices:
                closest = min(self.aperture_choices, key=lambda x: abs(x - value))
                msg = f'Aperture of {value} not supported, using closest option (or reformatting) to {closest}'
                print(msg)
                value = closest
            # gphoto2 only accepts strings formated as proper decimal numbers or integers without trailing zeros
            if value == int(value):
                value = int(value)
        return str(value), msg
    
    def pick_shutterspeed_value(self, value='AUTO'):
        '''
        Helper function to check and format the input used to change the shutter speed.
        Accepts slightly different inputs in PHOTO and VIDEO mode, so both are unified in this method.
        Input: Numeric string of the form '1/50' or '0.5' or '25', or int/float, or the string 'AUTO'.
        Output: the value (string) accepted by the camera, and a potential error message (string)
        '''
        if value is None:
            return None, 'Input is None, Shutterspeed unchanged. '
        
        msg = ''
        if value == 'AUTO':
            if self.mode == 0:
                value = 'bulb'
            else:
                value = 'auto'
        else:
            if value not in self.shutter_choices:
                num_choices = [eval(choice) for choice in self.shutter_choices] # convert all string options to numeric values                
                if type(value) == int or type(value) == float:
                    num_value = value
                else:
                    try:
                        num_value = eval(value)
                    except NameError:
                        msg = f"Value {value} not supported. Please use string 'AUTO' or a number (int/float/numeric string)."
                        print(msg)
                        print('Supported numeric values: ', self.shutter_choices)
                        return None, msg

                closest = self.shutter_choices[num_choices.index(min(num_choices, key=lambda x: abs(x - num_value)))]
                msg = f'Shutterspeed of {value} not supported, using closest (or reformatting) option of {closest}'
                print(msg)
                value = closest
        return value, msg

    def pick_continuous_AF_value(self, value='Off'):
        '''
        Helper function to check and format the input value used to change the continuous auto focus setting.
        Accepts slightly different inputs in PHOTO and VIDEO mode, so both are unified in this method.
        Input: string 'On' or 'Off', or int 1 or 0, or bool True or False
        Output: the value (string) accepted by the camera, the name of the configuration, and a potential error message (string)
        '''
        if value is None:
            return None, None, 'Input is None, Continuous AF setting unchanged.'
        if self.mode == 0:
            config = 'continuousaf'
        else:
            config = 'movieservoaf' # because the config is named differently in VIDEO mode

        value_dict = {0:'Off',1:'On', '0':'Off', '1':'On', 'False':'Off','True':'On','off':'Off','on':'On', 'Off':'Off', 'On':'On'} # gphoto2 only accepts the strings 'Off' and 'On' but this seems too restrictive
        if value not in value_dict:
            error_msg = f"Value {value} not supported. Please use 'Off' or 'On'."
            print(error_msg)
            return None, config, error_msg

        value = value_dict[value]
        return value, config, ''

    def reset_after_abort(self):
        '''
        Helper function to reset the camera configuration to a known state after a capture process was aborted.
        This should prevent the camera from getting stuck in an unknown or unexpected state.
        '''
        if self.mode == 0: # this refers to the mode at initialisation of this camera, not the current mode
            # The current mode might have been changed by the user, but we want to reset to the initial mode
            self.set_config_fire_and_forget('eosmoviemode', 0)
            self.set_config_fire_and_forget('eosremoterelease', 'None')
            self.set_config_fire_and_forget('drivemode', 'Single')
        else:
            self.set_config_fire_and_forget('movierecordtarget', 'None')
        return 'Reset completed'
        
    def capture_image(self, aperture=None, iso=None, shutterspeed=None, c_AF=None, download=True, target_path='.'):
        '''
        Top-level API call to capture a single image.
        Optionally change the capture parameters before starting the capture.
        Selects the correct capture function based on camera mode.
        Input: aperture, iso, shutterspeed, c_AF: see set_capture_parameters()
                download: bool, whether to download the image to the target path
                target_path: string, path to the directory where the image will be saved
        Output: file_path: string, msg: string
        '''

        # Check if the camera is in the correct mode
        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture static images"
            print(error_msg)
            return None, error_msg
        
        msgs = ''
        # Change capture parameters if requested
        input_params = [aperture, iso, shutterspeed, c_AF]
        if any(param is not None for param in input_params): # if any parameters are specified
            current_params = self.get_capture_parameters()
            current_params = list(current_params)
            target_params = [current_params[i] if item is None else item for i, item in enumerate(input_params)]

            msg = self.set_capture_parameters(*target_params)
            msgs += msg

        # Trigger the capture
        success, file_path, cam_path, msg = self.capture_immediate(download=download, target_path=target_path)
        msgs += msg
        time.sleep(0.8) # wait a moment to allow the camera to reset after the capture

        return file_path, cam_path, msgs
    
    def capture_video(self, aperture=None, iso=None, shutterspeed=None, c_AF=None, duration=1, target_path='.'):
        '''
        Top-level API call to capture a video.
        Selects the correct recording function based on camera mode.
        Optionally change the capture parameters before starting the recording.
        Input: aperture, iso, shutterspeed, c_AF: see set_capture_parameters()
                duration: float, duration of the recording in seconds
                target_path: string, path to the directory where the video will be saved
        Output: success: bool, file_path: string, msg: string
        '''

        msgs = ''
        # Change capture parameters if requested
        input_params = [aperture, iso, shutterspeed, c_AF]
        if any(param is not None for param in input_params):
            [current_params] = self.get_capture_parameters()
            new_params = [current_params[i] if item is None else item for i, item in enumerate(input_params)]
            msg = self.set_capture_parameters(*new_params)
            msgs += msg

        if self.mode == 0:
            success, file_path, msg = self.record_preview_video(t=duration, target_path=target_path, resolution_prio=True)
        else:
            success, file_path, msg = self.record_video(t=duration, download=True, target_path=target_path)
        msgs += msg
        return success, file_path, msgs


    ''' PHOTO mode only methods'''

    def set_exposure_manual(self):
        '''
        Set the camera's auto-exposure mode to manual, so that shutter, aperture, and iso can be set remotely.
        Only supported in PHOTO mode.
        '''

        if self.mode == 1:
            print("Camera must be in PHOTO mode to set exposure mode to manual")
            return False
        
        self.set_config_and_confirm(['autoexposuremodedial'], ['Fv']) # 'Fv' == Canon's 'Flexible-Priority Auto Exposure', useful for manual access
        return True
    
    def set_save_target(self):
        '''
        Set the camera's save target to the SD card, so that all captures are saved to the SD card by default.
        '''
        if self.mode == 1:
            print("Camera must be in PHOTO mode to set the save target for still images")
            return False
        
        self.set_config_and_confirm(['capturetarget'], ['Memory card']) # '1' == Memory card
        return
    
    def pick_iso_value(self, value='AUTO'):
        '''
        Helper function to check and format the input value used to change the ISO setting.
        Input: int, numeric string, or string 'AUTO'
        Output: the value (string) accepted by the camera, and a potential error message (string)
        '''
        if value is None:
            return None, 'Input is None, ISO unchanged. '
        
        msg = ''
        if value == 'AUTO':
            value = 'Auto'
        else:
            if type(value) == int or type(value) == float:
                value = round(value)
            elif type(value) == str:
                try:
                    value = round(eval(value))
                except NameError:
                    msg = f"Value {value} not supported. Please use string 'AUTO' or a number (int/float/numeric string)."
                    print(msg)
                    print('Supported numeric values: ', self.iso_choices)
                    return None, msg

            if value not in self.iso_choices:
                closest = min(self.iso_choices, key=lambda x: abs(x - value))
                msg = f'ISO of {value} not supported, using closest option of {closest}'
                print(msg)
                value = closest
            value = str(value)
        return value, msg
    
    def set_iso(self, value='AUTO'):
        '''
        Use this if you want to change ONLY the ISO setting.
        Always returns the (new) currently active setting.
        Only supported in PHOTO mode.
        '''
        msg = ''
        if self.mode == 1:
            msg = "Camera must be in PHOTO mode to manually set ISO."
            print(msg)
            return None, msg
        
        corrected_value, msg = self.pick_iso_value(value)
        if corrected_value is None:
            return self.get_iso(), msg
        self.set_config_and_confirm(['iso'], [corrected_value])
        current = self.get_iso()
        return current, msg

    def set_image_format(self, value=0, list_choices=False):
        '''
        Change the target image format, or optionally only list the available options.
        Always treturns the (new) currently active setting.
        Only supported in PHOTO mode.
        Input: value as int (choice index) or string (choice name)
        '''
        msg = ''
        if self.mode == 1:
            msg = "Camera must be in PHOTO mode to change the target image format"
            print(msg)
            return None, None, msg
        
        im_format = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'imageformat'))
        choices = list(im_format.get_choices())
        if list_choices:
            print(choices)
            msg = "Select format by full string or index"
            return im_format.get_value(), choices, msg
        if str(value) not in choices:
            if str(value).isnumeric() and int(value) < len(choices):
                value = choices[int(value)]
            else:
                msg = f"Format {value} not supported, please input choice either as full string or by index."
                print(msg)  
                return im_format.get_value(), choices, msg
        
        self.set_config_and_confirm(['imageformat'], [value])
        return value, choices, msg
    
    def trigger_AF(self, duration=0.2):
        '''
        Trigger auto-focus once. 
        It is currently not possible to check if focus has been achieved.
        This function might need to be called repeatedly to adjust focus.
        The duration determines how long the thread waits for the camera to try and focus. Depending on camera model this will be very short anyway, but we don't want to cut it off too early. Waiting longer than necessary is not a problem.
        (Equivalent to the bash command --set-config autofocusdrive=1)
        Only supported in PHOTO mode.
        Output: string describing the action taken
        '''
        if self.mode == 1:
            msg = "Camera must be in PHOTO mode to manually trigger auto focus"
            print(msg)
            return msg
        self.set_config_fire_and_forget('autofocusdrive', 1)
        # sleep to give the camera time to focus
        time.sleep(duration)
        self.set_config_fire_and_forget('autofocusdrive', 0)
        return 'AF triggered once'
    
    def set_AF_location(self, x=4096, y=2732):
        '''
        Set the auto focus point to a specific pixel location.
        (Equivalent to the bash command --set-config eoszoomposition=x,y)
        Only supported in PHOTO mode.
        Input: x and y are int, supported range is the image resolution, normally (1,1) to (8192,5464)
        '''
        msg = ''
        if self.mode == 1:
            msg = "Camera must be in PHOTO mode to manually set auto focus location"
            print(msg)
            return None, msg
        if type(x) != int or type(y) != int:
            msg = f"AF point {x},{y} not supported, please input values as integers."
            return None, msg
        
        AF_point = gp.check_result(gp.gp_widget_get_child_by_name(self.config, 'eoszoomposition'))
        if 0 <= x <= 8192 and 0 <= y <= 5464:
            self.set_config_fire_and_forget('eoszoomposition', f"{x},{y}")
            return f'{x},{y}', msg
        else:
            msg = f"AF point {x},{y} not supported, please input values between according to your selected image resolution, normally between 0 and 8192 for x and 0 and 5464 for y."
            return AF_point.get_value(), msg
    
    def show_live_preview(self, file_path='./live_preview.jpg'):
        '''
        Display preview frames on the PC until the user interrupts the preview with 'q'.
        Usually 960x640 at around 15 fps.
        The images are NOT saved on the device or pc.
        Note that the live preview is not available during capture. This function temporarily blocks the USB I/O. Stop the live preview before changing configurations or starting a capture.
        Only supported in PHOTO mode.'''
        if self.mode == 1:
            msg = "Camera must be in PHOTO mode to display live preview"
            print(msg)
            return msg
        
        from PIL import Image
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation 
        
        self.capture_preview(target_file=file_path)
        im = Image.open(file_path)
        ax1 = plt.subplot(111)
        im1 = ax1.imshow(im)

        def update_live_view(i):
            self.capture_preview(target_file=file_path)
            im = Image.open(file_path)
            im1.set_data(im)

        ani = FuncAnimation(plt.gcf(), update_live_view, interval=50)

        def close(event):
            if event.key == 'q':
                plt.close(event.canvas.figure)

        cid = plt.gcf().canvas.mpl_connect("key_press_event", close)
        print('Press q to quit')
        plt.show()
        return

    def capture_preview(self, target_file='./preview.jpg'):
        '''
        Capture a preview image (i.e. viewfinder frame, with the mirror up) and save it to the target file.
        The taken image is NOT saved on the device, only on the computer.
        Only supported in PHOTO mode.
        '''
        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture a preview"
            print(error_msg)
            return False, error_msg
        
        camera_file = gp.check_result(gp.gp_camera_capture_preview(self.camera))
        camera_file.save(target_file)
        return True, 'saved to computer'

    def capture_immediate(self, download=True, target_path='.'):
        '''
        Taken an immeditate capture, triggering the shutter but without triggering the auto-focus first.
        Image is saved to camera's storage device first, optionally download the image to the target path. 
        The file name will follow the camera's set naming convention.
        Returns a boolean indicating success, the file path if saved to PC, and a message.
        Only supported in PHOTO mode.
        '''

        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture static images"
            print(error_msg)
            return False, None, None, error_msg
        
        self.set_config_fire_and_forget('eosremoterelease', 'Immediate') # trigger shutter
        timeout = time.time() + 5
        while True:
            # potentially need to catch exceptions here in case the new file event is not caught by this wait loop
            # loop times out after 10 seconds
            event_type, event_data = self.camera.wait_for_event(1000)
            if event_type == gp.GP_EVENT_FILE_ADDED:
                if download:
                    cam_file = self.camera.file_get(event_data.folder, event_data.name, gp.GP_FILE_TYPE_NORMAL)
                    cam_file.save(target_path+'/'+event_data.name)
                    self.set_config_fire_and_forget('eosremoterelease', 'Release Full') # reset shutter
                    return True, target_path+'/'+event_data.name, event_data.folder+'/'+event_data.name, 'downloaded'
                else:
                    self.set_config_fire_and_forget('eosremoterelease', 'Release Full') # reset shutter
                    return True, None, None, 'saved to camera'
            
            elif time.time() > timeout:
                error_msg = "Waiting for new file event timed out, capture may have failed."
                print(error_msg)
                self.set_config_fire_and_forget(['eosremoterelease'], ['Release Full']) # reset shutter
                return False, None, None, error_msg

    def record_preview_video(self, t=1, target_path ='.', resolution_prio=False):
        '''
        Capture a series of previews (i.e. the viewfinder frames, with mirror up)
        for a duration of t seconds, pipe them directly to the PC save them as a video file on the PC.
        The file will not be saved to the camera's storage device.
        Note that this function will overwrite existing files in the specified location!
        Only supported in PHOTO mode.
        Inputs: t=duration in seconds (int or float), target_file=string with file path, resolution_prio=boolean
        '''
        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture preview videos"
            print(error_msg)
            return False, None, error_msg
        
        target_file = target_path + '/prev_vid.mp4'
        if os.path.exists(target_file): # always overwrite existing file to prevent ffmpeg error
            os.remove(target_file)

        # if a higher resolution is the priority, record in 'eosmoviemode' at 1024x576 and ~25 fps
        # if a higher frame rate is the priority, record at 960x640 and close to ~60 fps
        if resolution_prio:
            self.set_config_and_confirm(['eosmoviemode'], [1])
        else:
            self.set_config_and_confirm(['liveviewsize'], ['Large']) # set to max size: 960x640

        # Attempting to recreate the bash command "gphoto2 --capture-movie"
        # under the hood, this just takes repeated preview captures
        # see https://github.com/gphoto/gphoto2/blob/f632dcccfc2f27b7e510941335a80dfc986b4bf2/gphoto2/actions.c#L1053
        # sp.call(['gphoto2 --capture-movie=2s'], shell=True) # Can't call the bash command here, because I/O is busy
        #OK, path = gp.gp_camera_capture(self.camera, gp.GP_CAPTURE_MOVIE) # error: function not supported
        ffmpeg_command = [
            'ffmpeg', 
            '-f', 'image2pipe',           # Input format
            '-vcodec', 'mjpeg',
            '-i', '-',                    # Input comes from a pipe
            '-c:v', 'libx264',            # Video codec to use for encoding
            '-pix_fmt', 'yuvj422p',        # Output pixel format
            target_file                   # Output file path
        ]

        ffmpeg = Popen(ffmpeg_command, stdin=PIPE)

        start_time = time.time()  # Start the timer
        while True:
            if time.time() - start_time > t:
                break  # Stop recording after t seconds
            capture = self.camera.capture_preview()
            filedata = capture.get_data_and_size()
            data = memoryview(filedata)
            ffmpeg.stdin.write(data.tobytes())
        ffmpeg.stdin.close()
        ffmpeg.wait()

        if resolution_prio:
            self.set_config_and_confirm(['eosmoviemode'], [0])
        return True, target_file, 'saved to computer'
    

    def capture_burst(self, t=0.5, save_timeout=5, speed=0):
        '''
        Shoot a quick burst of full-scale images for a duration of t seconds.
        Should achieve about 8-9fps. 
        The image files are saved to the camera storage device first and must be donwloaded separately.
        Returns a list of file locations on the camera.
        Only supported in PHOTO mode.
        Input: t=duration in seconds (int or float), save_timeout=seconds to wait for new files, speed=0,1,2 (int)
        Outputs: success=boolean, files=list of strings, msg=string
        '''
        speeds = ['Continuous low speed', 'Continuous high speed', 'Super high speed continuous shooting']

        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture burst"
            print(error_msg)
            return False, None, error_msg

        # Set the drive mode to continuous shooting
        self.set_config_and_confirm(['drivemode'], [speeds[speed]])

        # start shooting but activating remote trigger
        start = time.time()
        self.set_config_fire_and_forget('eosremoterelease', 'Immediate')
        while time.time() - start < t:
            pass # wait for the desired duration
        # and turn the trigger OFF again
        self.set_config_fire_and_forget('eosremoterelease', 'Release Full')

        # after the burst is over, fetch all the files
        # this allows for faster shooting rather than saving files after each capture
        files=[]
        timeout = time.time() + save_timeout # the save timeout stops retrieving of files if no new file has been written for a while
        while True:
            event_type, event_data = self.camera.wait_for_event(100)
            if event_type == gp.GP_EVENT_FILE_ADDED:
                files.append(event_data.folder +'/'+ event_data.name)
                timeout = time.time() + save_timeout
            elif time.time() > timeout:
                break

        # Finally, set the drive mode back to individual captures
        self.set_config_and_confirm(['drivemode'], ['Single'])
        return True, files, 'saved to camera'
    
    def start_burst(self, speed=0):
        '''
        Same as above, but split so this can run continuously and does not need a specified duration.
        Make sure to call stop_burst() some time after this.
        Only supported in PHOTO mode.
        Input: speed=0 or 1 (int), 1 is faster
        '''
        speeds = ['Continuous low speed', 'Continuous high speed', 'Super high speed continuous shooting']
        if self.mode == 1:
            error_msg = "Camera must be in PHOTO mode to capture burst"
            print(error_msg)
            return False, None, error_msg

        # Set the drive mode to continuous shooting
        self.set_config_and_confirm(['drivemode'], [speeds[speed]])

        # start shooting but activating remote trigger
        start = time.time()
        self.set_config_fire_and_forget('eosremoterelease', 'Immediate')
        return True


    def stop_burst(self, save_timeout=5):
        '''
        See above.
        This ends the burst and fetches all resulting file locations.
        Images have to be downloaded separately.
        Only supported in PHOTO mode.
        '''
        self.set_config_fire_and_forget('eosremoterelease', 'Release Full')

        # after the burst is over, fetch all the files
        # this allows for faster shooting rather than saving files after each capture
        files=[]
        timeout = time.time() + save_timeout # the save timeout stops retrieving of files if no new file has been written for a while
        while True:
            event_type, event_data = self.camera.wait_for_event(100)
            if event_type == gp.GP_EVENT_FILE_ADDED:
                files.append(event_data.folder +'/'+ event_data.name)
                timeout = time.time() + save_timeout
            elif time.time() > timeout:
                break

        # Finally, set the drive mode back to individual captures
        self.set_config_and_confirm(['drivemode'], ['Single'])
        return True, files, 'saved to camera'


    ''' VIDEO mode only methods'''

    def start_rec_vid(self):
        if self.mode == 0:
            error_msg = "Camera must be in VIDEO mode to record full-res videos"
            print(error_msg)
            return False, None, error_msg
        
        self.set_config_fire_and_forget('movierecordtarget', 'Card')
        return
    
    def stop_rec_vid(self, download=True, target_path='.', save_timeout=5):
        if self.mode == 0:
            error_msg = "Camera must be in VIDEO mode to record full-res videos"
            print(error_msg)
            return False, None, error_msg
        
        self.set_config_fire_and_forget('movierecordtarget', 'None')
        # fetching the file
        timeout = time.time() + save_timeout
        if download:
            while True:
                # potential for errors if the new file event is not caught by this wait loop
                event_type, event_data = self.camera.wait_for_event(1000)
                if event_type == gp.GP_EVENT_FILE_ADDED:
                    cam_file = self.camera.file_get(event_data.folder, event_data.name, gp.GP_FILE_TYPE_NORMAL)
                    cam_file.save(target_path+'/'+event_data.name)
                    return True, target_path+'/'+event_data.name, 'File downloaded to PC'
                elif time.time() > timeout:
                    error_msg = "Warning: Waiting for new file event timed out, capture may have failed."
                    print(error_msg)
                    return True, None, error_msg
        return True, None, 'saved to camera'

    def record_video(self, t=1, download=True, target_path='.', save_timeout=5):
        '''
        Record a video for a duration of t seconds.
        Resolution and file formats are set in the camera's menu. Storage medium must be inserted.
        Only supported in VIDEO mode.
        The video is written to the camera's storage device first and downloaded to the PC afterwards.
        Inputs: t=duration in seconds (int or float), download=boolean, target_path=string
        Output: success=boolean, file_path=string, msg=string
        '''
        if self.mode == 0:
            error_msg = "Camera must be in VIDEO mode to record full-res videos"
            print(error_msg)
            return False, None, error_msg
        
        # recording
        start = time.time()
        self.set_config_fire_and_forget('movierecordtarget', 'Card')
        while time.time() - start < t:
            pass
        self.set_config_fire_and_forget('movierecordtarget', 'None')

        # fetching the file
        timeout = time.time() + save_timeout
        if download:
            while True:
                # potential for errors if the new file event is not caught by this wait loop
                event_type, event_data = self.camera.wait_for_event(1000)
                if event_type == gp.GP_EVENT_FILE_ADDED:
                    cam_file = self.camera.file_get(event_data.folder, event_data.name, gp.GP_FILE_TYPE_NORMAL)
                    cam_file.save(target_path+'/'+event_data.name)
                    return True, target_path+'/'+event_data.name, 'File downloaded to PC'
                elif time.time() > timeout:
                    error_msg = "Warning: Waiting for new file event timed out, capture may have failed."
                    print(error_msg)
                    return True, None, error_msg
        return True, None, 'saved to camera'

if __name__ == '__main__':

    cam1 = EOS()
    # cam1.show_live_preview()
    print("Camera initalised")
