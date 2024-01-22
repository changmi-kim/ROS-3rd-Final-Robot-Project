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
        count = 0

        while True:
            client_socket, self.address = self.server_socket.accept()
            print(f"{self.address}에서 연결됨")

            ClientThread(client_socket, self.address)
            count += 1
            print(count)


class ClientThread(Thread):
    def __init__(self, client_socket, address):
        Thread.__init__(self)
        self.client_socket = client_socket
        self.address = address
        self.window_name = f"Client {address[0]}:{address[1]}"  # 고유한 윈도우 이름 설정
        self.daemon = True
        self.start()


    def run(self):
        self.update_frame()


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
            try:
                length = self.receive_stream(self.client_socket, 16)
                if length:
                    stringData = self.receive_stream(self.client_socket, int(length))
                    data = np.frombuffer(stringData, dtype='uint8')
                    self.frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                    self.show_frame()
                    
            except Exception as e:
                print(f"Error: {e}, Client: {self.address}")
                pass


    def show_frame(self):
        try:
            cv2.imshow(self.window_name, self.frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.cleanup()

        except Exception as e:
            print(f"Error: {e}, Client: {self.address}")
            pass


    def cleanup(self):
        if self.client_socket.fileno() != -1:
            self.client_socket.close()

        cv2.destroyWindow(self.window_name)



if __name__ == '__main__':
    HOST = '192.168.1.7'  # 외부에서 접근 가능하도록 설정
    PORT = 3306       # 포트 번호
    video_stream_widget = VideoStreamWidget(HOST, PORT)