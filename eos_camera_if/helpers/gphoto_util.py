import gphoto2 as gp
import subprocess as sp

def choose_camera():
    """
    List all available cameras and let the user chose one,
    return the correct port address.
    """

    # Kill any existing gphoto processes to free up the USB ports for communication
    command = f'killall gvfsd-gphoto2 gvfs-gphoto2-volume-monitor'
    sp.call([command], shell=True)

    camera_list = list(gp.Camera.autodetect()) # Find all available cameras
    if not camera_list:
        print('No camera detected. Make sure it is connected and turned on.')
        return
    camera_list.sort(key=lambda x: x[0]) # Sort by port
    
# ask user to choose one
    for index, (name, addr) in enumerate(camera_list):
        print('{:d}:  {:s}  {:s}'.format(index, addr, name))
    choice = input('Please input number of chosen camera: ')
    try:
        choice = int(choice)
    except ValueError:
        print('Integer values only!')
        return
    if choice < 0 or choice >= len(camera_list):
        print('Number out of range')
        return
    # use chosen camera
    name, addr = camera_list[choice]
    return addr

def detect_EOS_cameras():
    """
    Detect all connected EOS cameras and return a list of their port addresses.
    """

    # Kill any existing gphoto processes to free up the USB ports for communication
    command = f'killall gvfsd-gphoto2 gvfs-gphoto2-volume-monitor'
    sp.call([command], shell=True)

    camera_list = list(gp.Camera.autodetect()) # Find all available cameras
    if not camera_list:
        print('No camera detected. Make sure it is connected and turned on.')
        return
    camera_list.sort(key=lambda x: x[0]) # Sort by port
    EOS_ports = [element [1] for element in camera_list if 'EOS' in element[0]]
    if len(EOS_ports) == 0:
        print('No EOS cameras detected.')
        return
    return EOS_ports


if __name__=='__main__':
    ports = detect_EOS_cameras()
    print(ports)