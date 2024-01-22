# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# class ImgPublisher(Node):
#     def __init__(self):
#         super().__init__('img_publisher')
#         self.publisher = self.create_publisher(Image, '/camera', 10)

#         time_period = 0.01
#         self.timer = self.create_timer(time_period, self.time_callback)
#         self.cap = cv2.VideoCapture(0)
#         self.cv_bridge = CvBridge()

#     def time_callback(self):
#         ret, frame = self.cap.read()
#         img = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
#         self.publisher.publish(img)
#         cv2.waitKey(1)



# def main() :
#     rclpy.init()
#     node = ImgPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__' :
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from aruco_processing import aruco_processing  # Import the aruco_processing function

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/v4l2_camera', 10)

        time_period = 0.01
        self.timer = self.create_timer(time_period, self.time_callback)
        self.cap = cv2.VideoCapture('/dev/video0')
        self.cv_bridge = CvBridge()

    def time_callback(self):
        ret, frame = self.cap.read()

        # Process the image using aruco_processing function
        processed_image = aruco_processing(frame)

        img = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.publisher.publish(img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
