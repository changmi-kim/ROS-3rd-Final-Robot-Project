import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int8MultiArray, Float32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from vision_object_detector_msgs.msg import DetectionResultArray
from cv_bridge import CvBridge
import cv2
import numpy
import math

class VisionAvoidance(Node):
    def __init__(self):
        super().__init__('vision_avoidance')
        self.cv_bridge = CvBridge()
        self.qos = QoSProfile(depth=10)

        pub_qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.subscriber_img = self.create_subscription(Image, '/debug_image', self.img_callback, qos_profile_sensor_data)
        self.subscriber_vision_detection_result = self.create_subscription(DetectionResultArray, '/detection_result', self.detection_result_callback, 10)
        self.subscriber_depth = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile=qos_profile_sensor_data)

        self.publisher_cmd_vel = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.publisher_front_degree_scan = self.create_publisher(LaserScan, '/front_degree_scan', pub_qos_profile)
        self.publisher_front_depth = self.create_publisher(Float32, '/front_depth', pub_qos_profile)
        self.publisher_debug_image_tracker = self.create_publisher(Image, '/debug_image_tracker', 10)

        self.i = 0
        self.twist = Twist()
        self.twist.linear.x = 0.3
        self.label = ''
        self.bbox_cx = 0
        self.bbox_cy = 0
        self.bbox_w = 0
        self.bbox_h = 0
        self.delta = 0
        self.min_distance = 0.0

    def img_callback(self, msg: Image):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg)

        # self.tracker()

        if self.label == 'person': 
            if self.bbox_w > 40 and self.bbox_h > 100:
                if self.min_distance < 0.3:
                    # self.publish_stop_cmd()
                    self.publish_back_cmd

    def detection_result_callback(self, msg):
        for detection in msg.detection_result:
            self.label = detection.label
            self.bbox_cx = detection.bbox_center_x
            self.bbox_cy = detection.bbox_center_y
            self.bbox_w = detection.bbox_size_x
            self.bbox_h = detection.bbox_size_y

    def lidar_callback(self, msg):
        # lidar_data = [msg.ranges[187], msg.ranges[156], msg.ranges[125], msg.ranges[94], msg.ranges[63]]
        front_data = msg.ranges[94:157]
        front_data_filtered = []

        for distance in front_data:
            if distance > 0:
                front_data_filtered.append(distance)

        if front_data_filtered:
            self.min_distance = min(front_data_filtered)
            self.publisher_front_depth.publish(Float32(data = self.min_distance))
        else:
            self.min_distance = float('inf')

        front_degree_msg = LaserScan()
        front_degree_msg.header = msg.header
        # front_degree_msg.angle_min = msg.angle_min + math.radians(22.5)
        # front_degree_msg.angle_max = msg.angle_min + math.radians(67.5)
        front_degree_msg.angle_min = msg.angle_min - math.radians(45 + 180)
        front_degree_msg.angle_max = msg.angle_min + math.radians(0 + 180)
        front_degree_msg.angle_increment = msg.angle_increment
        front_degree_msg.time_increment = msg.time_increment
        front_degree_msg.scan_time = msg.scan_time
        front_degree_msg.range_min = msg.range_min
        front_degree_msg.range_max = msg.range_max
        front_degree_msg.ranges = front_data

        self.publisher_front_degree_scan.publish(front_degree_msg)

    def tracker(self):
        h, w, d = self.cv_image.shape
        cv2.circle(self.cv_image, (int(self.bbox_cx), int(self.bbox_cy)), 20, (0, 0, 255), -1)
        
        err = self.bbox_cx - w / 2
        self.delta = err
        self.twist.angular.z = (-1) * self.delta / 100
        self.get_logger().info(f'angular.z = {self.twist.angular.z}')
        
        self.publisher_cmd_vel.publish(self.twist)
        self.publisher_debug_image_tracker.publish(self.cv_bridge.cv2_to_imgmsg(self.cv_image, encoding = 'bgr8'))
        
        cv2.imshow('debug_img_tracker', self.cv_image)
        cv2.waitKey(10)

        key = cv2.waitKey(1)
        if key == ord('q'):
            self.stop()

    def publish_stop_cmd(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot stopped. Front obstacle distance:{self.min_distance}')

    def publish_back_cmd(self):
        self.twist.linear.x = -0.5
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot has reversed. Front obstacle distance:{self.min_distance}')

def main(args=None):
    rclpy.init(args=args)
    
    vision_avoidance = VisionAvoidance()

    try:
        rclpy.spin(vision_avoidance)

    except KeyboardInterrupt:
        vision_avoidance.stop()
        vision_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()