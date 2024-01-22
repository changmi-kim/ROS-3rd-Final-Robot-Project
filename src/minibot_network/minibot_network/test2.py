import sys
import cv2
import numpy as np
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget
import socket
from threading import Thread

class CameraServerThread(Thread):
    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port

    def run(self):
        cap = cv2.VideoCapture(0)  # 서버에서는 카메라 인덱스를 조정하십시오.
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
        print(f"서버 대기 중... {self.host}:{self.port}")

        while True:
            client_socket, _ = server_socket.accept()
            print(f"클라이언트 연결됨: {self.host}:{self.port}")
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                
                # 영상 프레임을 JPEG 형식으로 인코딩하여 클라이언트에 전송
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                result, imgencode = cv2.imencode('.jpg', frame, encode_param)
                data = np.array(imgencode)
                string_data = data.tostring()
                client_socket.send(str(len(string_data)).ljust(16).encode())
                client_socket.send(string_data)
                
            client_socket.close()

class CameraViewer(QMainWindow):
    def __init__(self, host, port, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"라즈베리 파이 카메라")
        self.setGeometry(100, 100, 800, 600)

        self.host = host
        self.port = port

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen()

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QVBoxLayout(self.central_widget)

        self.label = QLabel(self)
        self.layout.addWidget(self.label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_image)
        self.timer.start(1000 // 30)  # 30 FPS

        self.image_received = False

    def update_image(self):
        try:
            client_socket, address = self.server_socket.accept()
            length = int(client_socket.recv(16))
            string_data = client_socket.recv(length)
            data = np.frombuffer(string_data, dtype='uint8')
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            self.label.setPixmap(pixmap)

            self.image_received = True


        except Exception as e:
            # 이미지를 수신하지 못한 경우, 여기에서 기다릴 수 있도록 로직 추가
            if not self.image_received:
                print("Waiting for image...")
            else:
                print(f"Error: {e}")
            pass

def main():
    app = QApplication(sys.argv)
    cameras = []

    # 라즈베리 파이 서버의 IP 주소와 포트 번호 설정
    raspberry_pis = [
        {"host": "192.168.1.7", "port": 3306},
        # 추가 라즈베리 파이 설정
    ]

    for pi in raspberry_pis:
        server_thread = CameraServerThread(pi["host"], pi["port"])
        server_thread.start()
        viewer = CameraViewer(pi["host"], pi["port"])
        viewer.show()
        cameras.append(viewer)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
