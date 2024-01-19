import cv2
import numpy as np
import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, Duration
from cv_bridge import CvBridge


class MyClient(Node):

    def __init__(self):
        super().__init__('minibot_cam_publish')

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.7', 3306)
        self.connect_to_server()

        self.qos_profile_ = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        deadline=Duration(seconds=0, nanoseconds=100000000),  # 예: 100ms의 데드라인
                        )

        self.image_publisher_ = self.create_publisher(Image, 'minibot2_image', self.qos_profile_)
        self.bridge_ = CvBridge()
        self.cap_ = cv2.VideoCapture(0) 


        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)

        self.width = self.get_parameter('width').value
        self.length = self.get_parameter('length').value
        
        ## 0~100에서 90의 이미지 품질로 설정 (default = 95)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        self.camera_callback()

    def camera_callback(self):
        while True:
            ret, frame = self.cap_.read()

            if ret:
                frame = cv2.resize(frame, (self.width, self.length))

                _, frame = cv2.imencode('.jpg', frame, self.encode_param)

                data = np.array(frame)
                stringData = data.tostring()

                self.client_socket.sendall((str(len(stringData))).encode().ljust(16) + stringData)


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
    
    client_node.cap_.release()  # Add this line to release the camera resource
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
