from threading import Thread
import cv2
import socket
import numpy as np


class VideoStreamWidget(object):
    def __init__(self, host, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen()
        print("서버 대기 중...")
        self.client_socket, self.address = self.server_socket.accept()
        print(f"{self.address}에서 연결됨")

        # 데이터 스트림 수신을 위한 스레드 시작
        self.thread = Thread(target=self.update_frame, args=())
        self.thread.daemon = True
        self.thread.start()

    def receive_stream(self, sock, count):
        # 바이트 문자열
        buf = b''

        while count:
            newbuf = sock.recv(count)

            if not newbuf: return None
            
            buf += newbuf
            count -= len(newbuf)

        return buf

    
    def update_frame(self):

        while True:
            length = self.receive_stream(self.client_socket, 16)
            stringData = self.receive_stream(self.client_socket, int(length))
            data = np.frombuffer(stringData, dtype = 'uint8')
            
            #data를 디코딩한다.
            self.frame = cv2.imdecode(data, cv2.IMREAD_COLOR)


    def show_frame(self):
        # 프레임 표시
        try:
            cv2.imshow('frame', self.frame)
            key = cv2.waitKey(1)

            if key == ord('q'):
                self.client_socket.close()
                cv2.destroyAllWindows()
                exit(1)

        except:
            self.client_socket.close()
            cv2.destroyAllWindows()
            exit(1)

if __name__ == '__main__':
    HOST = '192.168.1.7'  # 외부에서 접근 가능하도록 설정
    PORT = 3306       # 포트 번호
    video_stream_widget = VideoStreamWidget(HOST, PORT)
    while True:
        try:
            video_stream_widget.show_frame()
        except AttributeError:
            pass