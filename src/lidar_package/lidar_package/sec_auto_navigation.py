import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
import math
from geometry_msgs.msg import Twist

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        # Publisher QoS를 BEST_EFFORT로 설정
        pub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher = self.create_publisher(LaserScan, '/scan', pub_qos_profile)
        
        # Subscription QoS를 BEST_EFFORT로 설정
        sub_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            sub_qos_profile
        )

        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 1.0  # 1미터 이내의 객체는 장애물로 간주

    def laser_scan_callback(self, msg):
        # 정면 각도 범위를 좌우 45도로 설정합니다.
        front_angles_range = range(
            int((math.pi/4 - msg.angle_min) / msg.angle_increment),
            int((3 * math.pi/4 - msg.angle_min) / msg.angle_increment)
        )
        
        # 정면에 장애물이 있는지 확인합니다.
        self.obstacle_in_path = any(
            0 < msg.ranges[angle] < self.safe_distance for angle in front_angles_range
        )

        # 로깅과 navigate 함수 호출
        if self.obstacle_in_path:
            self.get_logger().info('Obstacle detected in front within safe distance.')
        else:
            self.get_logger().info('No obstacle detected in front within safe distance.')
        
        self.navigate()

    def navigate(self):
        cmd = Twist()
        if self.obstacle_in_path:
            self.stop_robot()
        else:
            self.move_robot_forward()
        self.cmd_publisher.publish(cmd)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)
        self.get_logger().info('Robot stopped.')

    def move_robot_forward(self):
        move_msg = Twist()
        move_msg.linear.x = 0.5
        move_msg.angular.z = 0.0
        self.cmd_publisher.publish(move_msg)
        self.get_logger().info('Robot moving forward.')

def main(args=None):
    rclpy.init(args=args)
    autonomous_navigation_node = AutonomousNavigationNode()
    rclpy.spin(autonomous_navigation_node)
    autonomous_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
