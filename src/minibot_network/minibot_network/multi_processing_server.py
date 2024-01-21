import socket
import multiprocessing
import cv2
import numpy as np

class MyServer:

    def __init__(self):
        self.client_sockets = []
        self.HOST = "192.168.1.7"
        self.PORT = 3306
        self.server_socket = None
        self.ip_name = {"192.168.1.14": "minibot1", "192.168.1.7": "minibot2"}
        self.image_queues = {ip: multiprocessing.Queue() for ip in self.ip_name}
        self.display_client = multiprocessing.Value('s', '')
    
    def image_process(self, client_socket, client_id, image_queue):
        while True:
            # 이미지 수신 및 처리 로직
            try:
                length = self.recvall(client_socket, 16)
                stringData = self.recvall(client_socket, int(length))
                data = np.frombuffer(stringData, dtype='uint8')
                frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                image_queue.put(frame)
            except Exception as e:
                break

    def run_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.HOST, self.PORT))
        self.server_socket.listen()
        print("Server is listening...")

        while True:
            client_socket, client_address = self.server_socket.accept()
            client_id = client_address[0]
            print(f'Connected by: {self.ip_name[client_id]}')

            # 클라이언트 별 프로세스 시작
            p = multiprocessing.Process(target=self.image_process, args=(client_socket, client_id, self.image_queues[client_id]))
            p.start()

    def user_input_thread(self):
        while True:
            client_id = input("보여줄 클라이언트 번호를 입력하세요 (숨기려면 -1): ")
            with self.display_client.get_lock():
                self.display_client.value = client_id

    def display_images(self):
        while True:
            with self.display_client.get_lock():
                client_id = self.display_client.value

            if client_id in self.image_queues and not self.image_queues[client_id].empty():
                frame = self.image_queues[client_id].get()
                cv2.imshow('ImageWindow', frame)
                cv2.waitKey(1)
            elif client_id == "-1":
                cv2.destroyAllWindows()

    def recvall(self, sock, count):
        buf = b''
        while count:
            newbuf = sock.recv(count)
            if not newbuf:
                return None
            buf += newbuf
            count -= len(newbuf)
        return buf


def main():
    server = MyServer()

    server_process = multiprocessing.Process(target=server.run_server)
    server_process.start()

    user_input_process = multiprocessing.Process(target=server.user_input_thread)
    user_input_process.start()

    display_process = multiprocessing.Process(target=server.display_images)
    display_process.start()

    server_process.join()
    user_input_process.join()
    display_process.join()

if __name__ == '__main__':
    main()
