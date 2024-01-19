import rclpy as rp
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException  # 외부에서 실행이 중단 되었을 경우 발생하는 오류
from rclpy.qos import QoSProfile  # QoS 프로파일 호출
from rclpy.qos import QoSReliabilityPolicy  # QoS 신뢰성 설정
# from rclpy.qos import qos_profile_sensor_data  # 센서 데이터 같이 빠르게 변하는 데이터 처리
from communication_msgs.msg import Talker
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class MinibotListener(Node):

    def __init__(self):
        super().__init__('Minibot_listener')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(Talker, 'command', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.talker_name}: {msg.message}")

def main(args=None):
    rclpy.init(args=args)
    listener = MinibotListener()

    try:
        rclpy.spin(listener)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


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


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # v4l2_camera 노드에서 발행하는 토픽
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        # 이미지 데이터 처리
        # GUI에 이미지 표시
        pass

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()