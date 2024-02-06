import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ctypes import *
import time

LIB_PATH = '../bin/linux-x86_64/libtsanpr.so'
print('LIB_PATH=', LIB_PATH)

lib = cdll.LoadLibrary(LIB_PATH)

lib.anpr_initialize.argtype = c_char_p
lib.anpr_initialize.restype = c_char_p

lib.anpr_read_pixels.argtypes = (c_char_p, c_int32, c_int32, c_int32, c_char_p, c_char_p, c_char_p)
lib.anpr_read_pixels.restype = c_char_p

class LicensePlateRecognitionNode(Node):
    def __init__(self):
        super().__init__('license_plate_recognition_node')
        self.bridge = CvBridge()

        lib.anpr_initialize(b'text')

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = cv_image.shape
        result = lib.anpr_read_pixels(
            bytes(cv_image), width, height, 0, b'BGR', b'text', b'')
        if len(result) > 0:
            self.get_logger().info(result.decode('utf8'))

        cv2.imshow("License_Plate_Recognition", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    license_plate_recognition_node = LicensePlateRecognitionNode()

    try:
        rclpy.spin(license_plate_recognition_node)
    except KeyboardInterrupt:
        pass

    license_plate_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
