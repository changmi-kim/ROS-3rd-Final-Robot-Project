import cv2
import rclpy
import socket
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge



class MyClient(Node):

    def __init__(self):
        super().__init__('minibot1')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.7', 3306)
        self.connect_to_server()

        self.qos_profile_ = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        )

        self.bridge_ = CvBridge()

        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)

        self.width = self.get_parameter('width').value
        self.length = self.get_parameter('length').value

        # self.image_subscriber_ = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_callback2, self.qos_profile_)
        self.image_subscriber_ = self.create_subscription(Image, 'minibot2_image', self.image_callback2, self.qos_profile_)
        # self.image_subscriber_ = self.create_subscription(Image, 'minibot2_image', self.image_callback, self.qos_profile_)


    
    def image_callback2(self, msg):
        try:
            # Convert the compressed image data to OpenCV format
            # np_arr = np.frombuffer(msg.data, np.uint8)
            # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Convert the normal image data to OpenCV format
            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
            cv_image = cv2.resize(cv_image, (self.width, self.length))

            # Display the image
            cv2.imshow("Image from ROS", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')



    
    def image_callback(self, msg):
        try:
            # 이미지 데이터를 ROS 메시지에서 numpy 배열로 변환
            cv_image = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')

            cv_image = cv2.resize(cv_image, (self.width, self.length))

            # 이미지를 JPEG 형식으로 다시 인코딩
            _, img_encoded = cv2.imencode('.jpg', cv_image)
            img_bytes = img_encoded.tobytes()

            # TCP/IP 소켓을 통해 이미지 데이터 전송
            try:
                # 이미지의 길이를 전송
                length = str(len(img_bytes)) + "\n"
                self.client_socket.sendall(length.encode('utf-8'))

                # 이미지 데이터를 전송
                self.client_socket.sendall(img_bytes)

                # CHUNK_SIZE = 1024  # 청크 크기를 1KB로 설정
                # for i in range(0, len(img_bytes), CHUNK_SIZE):
                #     chunk = img_bytes[i:i + CHUNK_SIZE]
                #     self.client_socket.sendall(chunk)


            except Exception as e:
                self.get_logger().error(f'Error sending image data: {e}')
                # 연결이 끊어졌을 경우 재연결 등의 처리를 할 수 있습니다.

            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')


    def connect_to_server(self):
        try:
            self.client_socket.connect(self.server_address)
            self.get_logger().info('Connected to the server')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to the server: {str(e)}')
            self.client_socket.close()
            rclpy.shutdown()
            exit(1)



def main(args=None):
    rclpy.init(args=args)
    client_node = MyClient()
    rclpy.spin(client_node)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
