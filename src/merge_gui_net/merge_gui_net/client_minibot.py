import cv2
import rclpy
import socket
import numpy as np
from rclpy.node import Node



class MyClient(Node):

    def __init__(self):
        super().__init__('minibot1')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.7', 2626)
        self.connect_to_server()

        self.cam_ = cv2.VideoCapture(0)

        ## 0~100에서 90의 이미지 품질로 설정 (default = 95)
        self.encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

        if self.cam_.isOpened():
            self.image_callback()
        
        else:
            self.get_logger().info("카메라 연결상태를 확인해 주세요.")


    def image_callback(self):
        while True:
            try:
                ret, frame = self.cam_.read()

                if not ret:
                    self.get_logger().info("카메라를 읽을 수 없습니다.")

                # encode_param의 형식으로 frame을 jpg로 이미지를 인코딩한다.
                result, frame = cv2.imencode('.jpg', frame, self.encode_param)

                # frame을 String 형태로 변환
                data = np.array(frame)
                stringData = data.tobytes()

                #서버에 데이터 전송
                #(str(len(stringData))).encode().ljust(16)
                self.client_socket.sendall((str(len(stringData))).encode().ljust(16) + stringData)

            except Exception as e:
                print(e)


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
