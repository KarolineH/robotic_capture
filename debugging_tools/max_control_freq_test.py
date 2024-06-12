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


    # Test 1: incremental movement via high-level cartesian pose control
    # starting_time = time.time()
    # for i in range(100):
    #     pose[2] = pose[2] + 0.0001 # increment the z position by 0.00001 meters
    #     action = data_io.create_cartesian_action(pose=pose)
    #     success &= IF.execute_action(action)
    #     print(time.time() - starting_time)

    # can also send a twist command
        # set speeds in cartesian frame
        # wait, stop when done
    # can also send joint speeds
        # set speed in joint frame
        # wait, stop when done

    # Test 2: incremental movement via high-level twist control

    from kortex_api.autogen.messages import Base_pb2
    import copy
    base = IF.base
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = 0
    twist.linear_y = 0
    twist.linear_z = 0.03
    twist.angular_x = 0
    twist.angular_y = 0
    twist.angular_z = 5

    command2 = copy.deepcopy(command)
    command2.twist.linear_z = -0.04

    starting_time = time.time()
    base.SendTwistCommand(command)
    time.sleep(0.01)
    print(IF.get_pose())
    base.SendTwistCommand(command2)
    time.sleep(0.01)
    print(IF.get_pose())
    base.SendTwistCommand(command)
    time.sleep(0.01)
    print(IF.get_pose())
    base.SendTwistCommand(command2)
    time.sleep(0.01)
    print(IF.get_pose())
    base.SendTwistCommand(command)
    time.sleep(0.01)
    print(time.time() - starting_time)
    base.Stop()