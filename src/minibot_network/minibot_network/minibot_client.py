import cv2
import rclpy
import socket
import numpy as np
from rclpy.node import Node

class MyClient(Node):

    def __init__(self):
        super().__init__('minibot1')
        self.HOST = "192.168.219.145"
        self.PORT = 5715
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = (self.HOST, self.PORT)
        self.connect_to_server()

        self.get_logger().info("서버 연결 성공!")


        self.declare_parameter('width', 320)    
        self.declare_parameter('length', 240)

        self.width = self.get_parameter('width').value
        self.length = self.get_parameter('length').value

        self.cap_ = cv2.VideoCapture(0)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        if not self.cap_.isOpened():
            self.get_logger().info("카메라 연결상태를 확인해주세요.")

        else:
            self.publish_frame()


    def publish_frame(self):
        self.get_logger().info("서버에 이미지를 전송합니다.")
        while True:
            try:
                ret, frame = self.cap_ .read()

                if not ret:
                    self.get_logger().info("비디오를 읽을 수 없습니다.")

                _, frame = cv2.imencode(".jpg", frame, self.encode_param)

                data = np.array(frame)
                stringData = data.tobytes()

                self.client_socket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

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
