import os

def choose_camera():
    os.listdir('/dev/')
    cams = [int(f.split('video')[1]) for f in os.listdir('/dev/') if f.startswith('video')]
    cams.sort()
    print(f'Available video devides: {cams}')
    choice = input('Please input number of chosen camera: ')
    try:
        choice = int(choice)
    except ValueError:
        print('Integer values only!')
        return
    if choice not in cams:
        print(f'Device {choice} not found.')
        return
    # use chosen camera
    name = f'/dev/video{choice}'
    return name

if __name__=='__main__':
    choose_camera()