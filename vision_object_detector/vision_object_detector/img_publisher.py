import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher = self.create_publisher(Image, 'image_raw', 10)

        # time_period = 0.01
        # self.timer = self.create_timer(time_period, 
        #                             self.time_callback)
        self.timer = self.create_timer(1.0 / 20, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0)
        self.cv_bridge = CvBridge()

        # self.width = 340
        # self.height = 240
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        self.declare_parameter('width', 320)
        self.width = self.get_parameter('width').value
        self.declare_parameter('length', 240)
        self.length = self.get_parameter('length').value

        output_msg = "Video Width : " + str(self.width) + "\n\r"
        output_msg = output_msg + "Video Length : " + str(self.length)
        self.get_logger().info(output_msg)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, (self.width, self.height))
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher.publish(image_msg)

def main():
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()