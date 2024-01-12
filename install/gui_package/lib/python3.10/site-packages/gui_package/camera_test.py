import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class PiCamSubscriber(Node):
    def __init__(self):
        super().__init__('pi_cam_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.imshow('Camera', image_np)
        cv2.waitKey(1)

def main(args=None):
    rp.init(args=args)
    pi_cam_subscriber = PiCamSubscriber()
    rp.spin(pi_cam_subscriber)
    pi_cam_subscriber.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
