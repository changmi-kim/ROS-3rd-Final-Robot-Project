import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class CamSubscriber(Node):
    
    def __init__(self):
        super().__init__('minibot_cam_subscriber')


        self.bridge_ = CvBridge()
        self.image_subscriber_ = self.create_subscription(Image, 'minibot_image', self.image_callback, 10)

    
    def image_callback(self, msg):
        # self.get_logger().info(f"받은 이미지의 인코딩: {msg}")
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환

            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')

            # 이미지를 화면에 표시
            cv2.imshow("Image from ROS", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {e}')



def main(args=None):
    rclpy.init(args=args)
    cam_subscriber_node = CamSubscriber()
    rclpy.spin(cam_subscriber_node)

    # Shutdown and clean up
    cam_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()