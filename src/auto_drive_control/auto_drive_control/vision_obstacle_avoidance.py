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
import datetime as dt
from enum import Enum

class VisionObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('vision_obstacle_avoidance')
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

        self.twist = Twist()
        self.twist.linear.x = 0.1

        self.img = None
        self.label = ''
        self.bbox_cx = 0
        self.bbox_cy = 0
        self.bbox_w = 0
        self.bbox_h = 0

        self.i = 0
        self.delta = 0
        self.min_distance = float('inf')

        self.obstacle_found = False
        self.waiting_start_time = None

        self.avoidance_move = False
        self.avoidance_state = 0  # WAITING = 0, STEP_ASIDE = 1, GO_STRAIGHT = 2, STEP_IN = 3, BACK = 4
        self.avoidance_angle_sign = 1
        self.avoidance_start_time = None
        self.avoidance_start_delta = 0

    def img_callback(self, msg: Image):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        self.tracker()

        if self.obstacle_found: return

        # self.twist.linear.x = 0.1
        # self.publisher_cmd_vel.publish(self.twist)

        # if self.label == 'person': 
        #     if self.bbox_w > 40 and self.bbox_h > 100:
        #         if self.min_distance < 0.3:
        #             self.publish_cmd_stop()
        #             # self.publish_cmd_back()

    def detection_result_callback(self, msg):
        for detection in msg.detection_result:
            self.label = detection.label
            self.bbox_cx = detection.bbox_center_x
            self.bbox_cy = detection.bbox_center_y
            self.bbox_w = detection.bbox_size_x
            self.bbox_h = detection.bbox_size_y

    def lidar_callback(self, msg: LaserScan):
        # lidar_data = [msg.ranges[187], msg.ranges[156], msg.ranges[125], msg.ranges[94], msg.ranges[63]]
        # front_data = msg.ranges[94:156]  # 전방 90도
        # front_data = msg.ranges[110:141]  # 전방 45도
        # front_data = msg.ranges[115:135]  # 전방 30도

        total_angles = 180
        center_index = 125
        angle_range = 45

        half_angle_range = angle_range / 2.0
        start_angle = center_index - half_angle_range
        end_angle = center_index + half_angle_range

        if start_angle < 0:
            start_angle += total_angles
        if end_angle >= total_angles:
            end_angle -= total_angles

        start_index = int(start_angle)
        end_index = int(end_angle)

        front_data = msg.ranges[start_index:end_index]
        front_data_filtered = []

        for distance in front_data:
            if distance > 0:
                front_data_filtered.append(distance)

        if front_data_filtered:
            self.min_distance = min(front_data_filtered)
            self.publisher_front_depth.publish(Float32(data = self.min_distance))
        else:
            self.min_distance = float('inf')

        self.get_logger().info(f'Front obstacle distance:{self.min_distance}')

        front_degree_msg = LaserScan()
        front_degree_msg.header = msg.header
        front_degree_msg.angle_min = msg.angle_min - math.radians(45 + 180)
        front_degree_msg.angle_max = msg.angle_min + math.radians(0 + 180)
        front_degree_msg.angle_increment = msg.angle_increment
        front_degree_msg.time_increment = msg.time_increment
        front_degree_msg.scan_time = msg.scan_time
        front_degree_msg.range_min = msg.range_min
        front_degree_msg.range_max = msg.range_max
        front_degree_msg.ranges = front_data

        self.publisher_front_degree_scan.publish(front_degree_msg)

        if not self.obstacle_found and (0.0 < self.min_distance < 0.3):
            self.publish_cmd_stop() 
            self.obstacle_found = True

            if self.waiting_start_time is None:
                self.waiting_start_time = dt.datetime.now()

            self.get_logger().info(f'Front obstacle distance:{self.min_distance}')

        if self.obstacle_found:
            if self.waiting_start_time is None: return

            if self.avoidance_state == 0 and self.min_distance > 0.5:
                self.obstacle_found = False
                self.waiting_start_time = None
                return
            
            if self.avoidance_state == 0 and self.min_distance < 0.2:
                self.avoidance_state = 4

            if self.avoidance_state == 4:
                self.waiting_start_time = dt.datetime.now()
                self.publish_cmd_back()

            if self.avoidance_state == 4 and self.min_distance > 0.3: 
                self.publish_cmd_stop()
                self.avoidance_state = 0
                self.waiting_start_time = dt.datetime.now()

            elapsed_time = (dt.datetime.now() - self.waiting_start_time ).total_seconds()
            self.get_logger().debug(f'elapsed time = {elapsed_time}')

            if self.avoidance_move is False and elapsed_time > 4.0:
                if self.min_distance < 0.35:
                    self.publish_cmd_back()
                    return
                
                self.avoidance_move = True
                self.avoidance_state = 1
                self.avoidance_angle_sign = 1 if self.delta > 0.0 else -1
                self.avoidance_start_time = dt.datetime.now()
                self.avoidance_start_delta = self.delta
                self.get_logger().info(f'delta = {self.delta}')

            if self.avoidance_move:
                if self.avoidance_state == 1:
                    self.publish_cmd_step_aside()
                    elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()

                    if elapsed_time > 2.0:
                        print('State changed.')
                        # self.avoidance_state = 2
                        self.avoidance_state = 3
                        self.avoidance_start_time = dt.datetime.now()

                # elif self.avoidance_state == 2:
                #     self.publish_cmd_go_straight()
                #     elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                #     max_time = 2.5 if abs(self.avoidance_start_delta) > 25 else 4.0

                #     if elapsed_time > max_time:
                #         self.avoidance_state = 3
                #         self.avoidance_start_time = dt.datetime.now()

                elif self.avoidance_state == 3:
                    self.publish_cmd_step_in()
                    elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                    
                    if elapsed_time > 2.0:
                        self.obstacle_found = False
                        self.waiting_start_time = None

                        self.avoidance_move = False
                        self.avoidance_state = 0
                        self.avoidance_start_time = None
                        
    def tracker(self):
        h, w, d = self.cv_image.shape
        cv2.circle(self.cv_image, (int(self.bbox_cx), int(self.bbox_cy)), 20, (0, 0, 255), -1)
        
        err = self.bbox_cx - w / 2
        self.delta = err
       
        # self.publish_cmd_for_tracker()

        self.publisher_debug_image_tracker.publish(self.cv_bridge.cv2_to_imgmsg(self.cv_image, encoding = 'rgb8'))
        
        cv2.imshow('debug_img_tracker', self.cv_image)
        cv2.waitKey(10)

        key = cv2.waitKey(1)
        if key == ord('q'):
            self.stop()

    def publish_cmd_for_tracker(self):
        self.twist.angular.z = (-1) * self.delta / 100
        self.get_logger().info(f'angular.z = {self.twist.angular.z}')
        self.publisher_cmd_vel.publish(self.twist)

    def publish_cmd_go_straight(self):
        self.twist.linear.x = 0.1
        # self.twist.angular.z = ((-1) * self.avoidance_angle_sign * 0.15) if (abs(self.avoidance_start_delta) > 25) else ((-1) * self.avoidance_angle_sign * 0.3)
        self.twist.angular.z = 0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot moved forward.')

    def publish_cmd_back(self):
        self.twist.linear.x = -0.1
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot has reversed.')
        self.get_logger().info(f'Front obstacle distance:{self.min_distance}')

    def publish_cmd_stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot stopped.')
        self.get_logger().info(f'Front obstacle distance:{self.min_distance}')

    def publish_cmd_step_aside(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = self.avoidance_angle_sign * (0.475 if abs(self.avoidance_start_delta) > 25 else 0.3)
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot is currently navigating around obstacles.')

    def publish_cmd_step_in(self):
        self.twist.linear.x = 0.2
        self.twist.angular.z = (-1) * self.avoidance_angle_sign * (0.475 if abs(self.avoidance_start_delta) > 25 else 0.3)
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot is re-entering the path.')

def main(args=None):
    rclpy.init(args=args)
    
    vision_obstacle_avoidance = VisionObstacleAvoidance()

    try:
        rclpy.spin(vision_obstacle_avoidance)

    except KeyboardInterrupt:
        vision_obstacle_avoidance.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()