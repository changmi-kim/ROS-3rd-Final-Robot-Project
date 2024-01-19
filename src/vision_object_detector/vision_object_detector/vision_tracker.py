import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from vision_object_detector_msgs.msg import DetectionResultArray
from cv_bridge import CvBridge
import cv2
import numpy
import math

class VisionTracker(Node):
    def __init__(self):
        super().__init__('vision_tracker')
        self.qos = QoSProfile(depth=10)
        self.cv_bridge = CvBridge()

        self.subscriber_img = self.create_subscription(Image, '/debug_image', self.img_callback, qos_profile_sensor_data)
        self.subscriber_vision_detection_result = self.create_subscription(DetectionResultArray, '/detection_result', self.detection_result_callback, 10)

        self.publisher_cmd_vel = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.publisher_debug_image_tracker = self.create_publisher(Image, '/debug_image_tracker', 10)

        self.i = 0
        self.twist = Twist()
        self.twist.linear.x = 0.3
        self.bbox_cx = 0
        self.bbox_cy = 0
        self.delta = 0.0

    def img_callback(self, msg: Image):
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
        h, w, d = cv_image.shape
        cv2.circle(cv_image, (int(self.bbox_cx), int(self.bbox_cy)), 20, (0, 0, 255), -1)
        
        err = self.bbox_cx - w / 2
        self.delta = err
        self.twist.angular.z = (-1) * self.delta / 100
        self.get_logger().info(f'angular.z = {self.twist.angular.z}')
        
        self.publisher_cmd_vel.publish(self.twist)
        self.publisher_debug_image_tracker.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding = msg.encoding))
        
        cv2.imshow('debug_img_tracker', cv_image)
        cv2.waitKey(10)

        key = cv2.waitKey(1)
        if key == ord('q'):
            self.stop()

    def detection_result_callback(self, msg):
        for detection in msg.detection_result:
            self.bbox_cx = detection.bbox_center_x
            self.bbox_cy = detection.bbox_center_y

    def publish_stop_cmd(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    
    vision_tracker = VisionTracker()

    try:
        rclpy.spin(vision_tracker)

    except KeyboardInterrupt:
        vision_tracker.stop()
        vision_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()