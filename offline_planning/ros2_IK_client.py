
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK

class IKClient(Node):
    ''' 
    A minimal ROS2 client node
    Can be used to calculate inverse kinematics using the simulated Kinova robot with MoveIt
    ROS2 must be installed and the Kinova MoveIt simulation (gen3_6dof_vision_moveit_config with fake hardware) must be running!
    '''
    def __init__(self):
        super().__init__('IKClient')
        self.cli = self.create_client(GetPositionIK, 'compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPositionIK.Request()

    def send_request(self, x ,y, z, qx, qy, qz, qw, link='dslr_body_link'):
        '''
        Inputs: Desired end-effector pose given as translation (x,y,z) and quaternion (qx,qy,qz,qw).
        '''
        
        self.req.ik_request.group_name = 'manipulator'
        self.req.ik_request.ik_link_name = link
        self.req.ik_request.pose_stamped.pose.position.x = x
        self.req.ik_request.pose_stamped.pose.position.y = y
        self.req.ik_request.pose_stamped.pose.position.z = z
        self.req.ik_request.pose_stamped.pose.orientation.x = qx
        self.req.ik_request.pose_stamped.pose.orientation.y = qy
        self.req.ik_request.pose_stamped.pose.orientation.z = qz
        self.req.ik_request.pose_stamped.pose.orientation.w = qw
        self.req.ik_request.avoid_collisions = True

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        error = self.future.result().error_code.val
        joint_angles = self.future.result().solution.joint_state.position 
        return error, joint_angles

def main(args=None):
    rclpy.init(args=args)
    ikcli = IKClient()
    response = ikcli.send_request(0.19157, 0.043617, 0.34076, 0.48835, 0.51191, -0.50606, 0.49332)
    ikcli.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()