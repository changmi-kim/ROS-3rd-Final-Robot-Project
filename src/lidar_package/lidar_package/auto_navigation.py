import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy, HistoryPolicy
import math
from geometry_msgs.msg import Twist
import time

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

        self.cmd_publisher = self.create_publisher(Twist, 'base_controller/cmd_vel_unstamped', 10)
        self.safe_distance = 0.2  # 1미터 이내의 객체는 장애물로 간주

    def laser_scan_callback(self, msg):
        
        # 정면 각도 범위를 좌우 90도로 설정합니다.
        front_angles_range = range(
            int((-1 * math.pi/4 - msg.angle_min) / msg.angle_increment),
            int((math.pi/4 - msg.angle_min) / msg.angle_increment)
        )
        
        # 유효한 거리 데이터만 필터링합니다.
        valid_ranges = [
            msg.ranges[angle] for angle in front_angles_range
            if msg.range_min < msg.ranges[angle] < msg.range_max
        ]

        # 정면에 장애물이 있는지 확인합니다.
        obstacle_detected = any(
            distance < self.safe_distance for distance in valid_ranges
        )
        
        if obstacle_detected:
            # 장애물과의 거리와 각도를 계산합니다.
            min_distance = min(valid_ranges)
            min_angle_index = valid_ranges.index(min_distance) 
            min_angle = front_angles_range[min_angle_index] * msg.angle_increment + msg.angle_min

            

             # 로거를 사용하여 장애물 정보를 출력합니다.
            self.get_logger().info(f'Obstacle detected! Distance: {min_distance:.2f} m, Angle: {math.degrees(min_angle):.2f} degrees')
        
        # 로직 후에 navigate를 호출하여 움직임을 결정합니다.
        self.navigate(obstacle_detected)

    # def handle_obstacle(self, angle, distance):
        
    # 외부 한 바퀴 돌기 로직 
    def navigate(self, obstacle_detected):
        if obstacle_detected:
            
            # 장애물이 감지되었으므로 로봇을 멈춥니다.
            self.stop_robot()


            self.left_rotation_robot()
        else:
            # 장애물이 없으므로 로봇이 계속 직진합니다.
            self.move_robot_forward()

    def stop_robot(self):
        # 로봇을 멈추기 위해 Twist 메시지를 발행합니다.
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_publisher.publish(stop_msg)
        self.get_logger().info('Robot stopped due to an obstacle!')

    def move_robot_forward(self):
        # 로봇을 전진시키기 위해 Twist 메시지를 발행합니다.
        move_msg = Twist()
        move_msg.linear.x = 0.1 # 생각보다 많이 느림 - 감지에 좋음 
        move_msg.angular.z = 0.0
        self.cmd_publisher.publish(move_msg)
        self.get_logger().info('Robot moving forward.')

    def left_rotation_robot(self):
        # 로봇을 회전 시키기 위해 Twist 메세지를 발행합니다.
        left_rotate_msg = Twist()
        left_rotate_msg.linear.x = 0.0
        left_rotate_msg.angular.z = 0.3 # 90회전 
        self.cmd_publisher.publish(left_rotate_msg)
        self.get_logger().info('Robot left moving forward.')

    def right_rotation_robot(self):
        # 로봇을 회전 시키기 위해 Twist 메세지를 발행합니다.
        right_rotate_msg = Twist()
        right_rotate_msg.linear.x = 0.0
        right_rotate_msg.angular.z = -0.3
        self.cmd_publisher.publish(right_rotate_msg)
        self.get_logger().info('Robot right moving forward.')

def main(args=None):
    rclpy.init(args=args)
    autonomous_navigation_node = AutonomousNavigationNode()
    rclpy.spin(autonomous_navigation_node)
    autonomous_navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
