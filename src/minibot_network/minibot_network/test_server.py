from threading import Thread
import cv2
import socket
import numpy as np
import queue



# 사용자 입력 처리 스레드
class UserInputThread(Thread):
    def __init__(self, client_threads):
        Thread.__init__(self)
        self.client_threads = client_threads
        self.daemon = True

    def run(self):
        while True:
            command = input("카메라 번호 (1, 2, 3)를 입력하세요: ")
            if command.isdigit():
                cam_number = int(command)
                if 1 <= cam_number <= len(self.client_threads):
                    ClientThread.deactivate_all_streams(self.client_threads)
                    self.client_threads[cam_number - 1].activate()



class VideoStreamWidget(object):
    def __init__(self, host, port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen()
        print("서버 대기 중...")
        count = 0
        
    def wait_for_client(self):
        client_socket, address = self.server_socket.accept()
        return client_socket, address


class ClientThread(Thread):
    def __init__(self, client_socket, address):
        Thread.__init__(self)
        self.client_socket = client_socket
        self.address = address
        self.window_name = f"Client {address[0]}:{address[1]}"  # 고유한 윈도우 이름 설정
        self.daemon = True

        print(f"{address}에서 연결됨")

        self.active = True 


    @classmethod
    def deactivate_all_streams(cls, client_threads):
        for thread in client_threads:
            thread.deactivate()

    def run(self):
        while True:
            if self.active:
                self.update_frame()

    
    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False


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
        if self.frame is not None and self.frame.size > 0:
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



# 메인 함수
if __name__ == '__main__':
    HOST = '192.168.1.7'
    PORT = 3306
    video_stream_widget = VideoStreamWidget(HOST, PORT)
    client_threads = []

    for _ in range(2):
        client_socket, address = video_stream_widget.wait_for_client()
        client_thread = ClientThread(client_socket, address)
        client_thread.start()
        client_threads.append(client_thread)
        client_thread.deactivate()  # 초기에는 모든 스트림을 비활성화합니다.

    user_input_thread = UserInputThread(client_threads)
    user_input_thread.start()