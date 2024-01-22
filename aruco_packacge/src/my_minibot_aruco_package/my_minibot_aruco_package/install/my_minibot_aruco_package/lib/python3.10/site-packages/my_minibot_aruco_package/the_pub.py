import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel
from pose_estimation_test2_pinkbot import PoseEstimationNode
class_PoseEstimation = PoseEstimationNode()

class MinibotPublisher(Node):

    def __init__(self):
        super().__init__('minibit_publisher')
        self.publisher_ = self.create_subscription(String, 'test', 10)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.callback)
        self.x, self.y, self.z = 0, 0, 3  # 예시 값


    def set_position(self, x, y, z):
        
        
        self.x = x
        self.y  = y
        self.z = z

    def callback(self):
        msg = String()  
        msg.data = f"X: {self.x}, Y: {self.y}, Z: {self.z}"   #"topic data"  
        self.publisher_.publish(msg) 
        print(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = MinibotPublisher()

    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel
# from std_msgs.msg import String

# class MinibotPublisher(Node):

#     def __init__(self):
#         super().__init__('minibit_publisher')
#         self.publisher_ = self.create_publisher(CmdAndPoseVel, '/aruco_result', 10)
#         timer_period = 0.5 
#         self.timer = self.create_timer(timer_period, self.callback)
#         self.aruco_result = CmdAndPoseVel()

#         # 예시 값 초기화
#         self.aruco_result.cmd_vel_linear = 0.1
#         self.aruco_result.cmd_vel_angular = 0.2
#         self.aruco_result.pose_x = 1.0
#         self.aruco_result.pose_y = 2.0
#         self.aruco_result.linear_vel = 0.3
#         self.aruco_result.angular_vel = 0.4
#         # self.aruco_result.str number = number
#         # self.aruco_result.center_x = center_x
#         # self.aruco_result.center_y = center_y
#         # self.aruco_result.distance_to_marker = z


#     def callback(self):
#         self.publisher_.publish(self.aruco_result)

# def main(args=None):
#     rclpy.init(args=args)
#     node = MinibotPublisher()

#     rclpy.spin(node)
#     node.destroy_node()
    
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()