import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel
from cv_bridge import CvBridge, CvBridgeError
import cv2
# import numpy
# import math

class VisionObjectTracker(Node):
    def __init__(self):
        super().__init__('vision_object_tracker')
        self.qos = QoSProfile(depth=10)

        self.subscription = self.create_subscription(CompressedImage, 'image_raw/compressed', self.image_callback, 10)
        self.subscriber_aruco_result = self.create_subscription(CmdAndPoseVel, '/aruco_result', self.aruco_result_callback, 10)

        self.publisher_cmd_vel = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.publisher_debug_image_aruco_tracker = self.create_publisher(Image, '/debug_image_aruco_tracker', 10)

        self.twist = Twist()
        self.twist.linear.x = 0.3
        self.bbox_cx = 0
        self.bbox_cy = 0
        self.delta = 0.0

    def aruco_result_callback(self, msg):

        self.aruco_result = CmdAndPoseVel()
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        except CvBridgeError as e:
            print(e)

        x, y, z = cv_image.shape
        cv2.circle(cv_image, (int(self.bbox_cx), int(self.bbox_cy)), 20, (0, 0, 255), -1)
        
        err = self.bbox_cx - x / 2
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

    def aruco_result_callback(self, msg):
        self.bbox_cx = msg.aruco_result.center_x
        self.bbox_cy = msg.aruco_result.center_y
        
    def publish_stop_cmd(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.twist)
        self.get_logger().info(f'Robot stopped.')

def main(args=None):
    rclpy.init(args=args)
    
    vision_object_tracker = VisionObjectTracker()

    try:
        rclpy.spin(vision_object_tracker)

    except KeyboardInterrupt:
        vision_object_tracker.stop()
        vision_object_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()