import numpy as np
from math import pi
from visual_kinematics.RobotSerial import RobotSerial, Frame
from scipy.spatial.transform import Rotation as Rotation

class robot_planner():
    def __init__(self):
        ''' Robot definition '''
        a = [0,0,410,0,0,0,0]
        d = [0,-(156.43+128.38), -5.38, -6.38, -(208.43+105.93), 0, -(105.93+61.53)]
        alpha = [pi,pi/2, pi, pi/2, pi/2, pi/2, pi]
        theta = [0,0, -0.5*pi, -0.5*pi, pi, pi, pi]

        self.dh_params = np.array([d, a, alpha, theta]).T
        self.robot_serial_obj = RobotSerial(self.dh_params)
        self.num_joints = 6

    def forward(self, joint_angles):
        ''' Takes 6 joint angles and returns the end effector pose (homogeneous transformation matrix) '''
        in_vector = [0,*joint_angles] 
        f = self.robot_serial_obj.forward(in_vector)
        return f

    def inverse(self, pose):
        xyz = pose[:3]
        abc = np.array([0.5 * pi, 0., pi])
        end = Frame.from_euler_3(abc, xyz)
        robot.inverse(end)
        # takes the end effector pose and returns the 6 joint angles
        # this is the inverse kinematics
        return joint_angles
    
    def kinova_to_vk(self, pose):
        ''' Converts a pose measured by the Kinova robot to the correct format read by visual_kinematics library.
        Input is a 1x6 vector [x,y,z,theta_x,theta_y,theta_z], that is Euler angles in x,y,z order and extrinsic convention, given in degrees.
        Output is a Frame object, which contains and 1x3 Euler angles in Z,Y,X order and intrinsic convention, given in radians.
        '''
        # TODO the conversion of Euler angles can be done much simpler, they are just permuted...?
        # TODO check that the radians and degrees are correct

        r = Rotation.from_euler('xyz', pose[3:], degrees=True)
        vk_angles = r.as_euler('ZYX', degrees=False)
        vk_angles = np.reshape(vk_angles, (1, -1))
        translation = np.reshape(pose[:3], (-1, 1))
        f = Frame.from_euler_3(vk_angles, translation)
        return f

    def vk_to_kinova(self, pose):
        ''' Converts a pose computed by the visual_kinematics library to the correct format read by the Kinova robot.
        Input is a Frame object, which contains Euler angles in Z,Y,X order and intrinsic convention and given in radians.
        Output is a 1x6 vector [x,y,z,theta_x,theta_y,theta_z], that is Euler angles in x,y,z order and extrinsic convention, given in degrees.
        '''
        r = Rotation.from_euler('ZYX', pose.euler_3, degrees=False)
        kinova_angles = r.as_euler('xyz', degrees=True)
        return np.append(pose.t_3_1.T, kinova_angles)    

    
if __name__ == '__main__':
    robot = robot_planner()
    joint_angles = [0,0,0,0,0,0]
    pose = robot.forward(joint_angles)
    rev = robot.vk_to_kinova(pose)
    pose2 = robot.kinova_to_vk(rev)
    print(pose)