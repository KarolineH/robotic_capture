import kinova_arm_if.helpers.kortex_util as k_util
import kinova_arm_if.helpers.data_io as data_io
from kinova_arm_if.arm_if import Kinova3
import time

args = k_util.parseConnectionArguments()
with k_util.DeviceConnection.createTcpConnection(args) as router:
    IF = Kinova3(router)
    success = True

    starting_pose = IF.get_pose() # [x,y,z,theta_x,theta_y,theta_z]
    pose = starting_pose
    starting_time = time.time()
    for i in range(100):
        pose[2] = pose[2] + 0.0001 # increment the z position by 0.00001 meters
        action = data_io.create_cartesian_action(pose=pose)
        success &= IF.execute_action(action)
        print(time.time() - starting_time)

    # can also send a twist command
        # set speeds in cartesian frame
        # wait, stop when done
    # can also send joint speeds
        # set speed in joint frame
        # wait, stop when done