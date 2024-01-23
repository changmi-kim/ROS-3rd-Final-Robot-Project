import socket
import threading
import cv2
import numpy as np



class MyServer():

    def __init__(self):
        super().__init__()
        self.client_sockets = []
        self.client_sockets_lock = threading.Lock()
        # self.HOST = "192.168.1.7" # 로컬 IP 주소
        # self.PORT = 3306
        self.HOST = "192.168.219.145"
        self.PORT = 5715
        self.server_socket = None

        self.client_thread_count = 0
        self.image_thread_count = 0

        self.ip_name = {"192.168.1.14" : "minibot1",
                        "192.168.1.7" : "minibot2",
                        "192.168.1.6" : "minibot3",
                        }
        
        self.images = {}  # 클라이언트별 최신 이미지를 저장하는 딕셔너리
        self.display_client = None  # 현재 표시중인 클라이언트

        self.thread_stop_flags = {}  # 쓰레드들 플레그 확인
        self.threads = {}  # 스레드 참조를 저장할 딕셔너리
                        


            
    def image_callback(self, client_socket, client_id, stop_event):
        self.image_thread_count += 1
        print("생성된 이미지 쓰레드 갯수 : ", self.image_thread_count)
        try:
            while not stop_event.is_set():
            
                # client에서 받은 stringData의 크기 (==(str(len(stringData))).encode().ljust(16))
                length = self.recvall(client_socket, 16)
                stringData = self.recvall(client_socket, int(length))
                data = np.frombuffer(stringData, dtype='uint8')
                
                #data를 디코딩한다.
                frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

                self.images[client_id] = frame  # 클라이언트 별로 이미지 저장

        finally:  
            client_socket.close()
            self.client_thread_count -= 1
            self.image_thread_count -= 1

            print("생성된 고객 쓰레드 갯수 : ", self.client_thread_count)
            print("생성된 이미지 쓰레드 갯수 : ", self.image_thread_count)

            with self.client_sockets_lock:
                if client_socket in self.client_sockets:
                    self.client_sockets.remove(client_socket)

                print(f'남은 클라이언트 수: {len(self.client_sockets)}')


    #socket에서 수신한 버퍼를 반환하는 함수
    def recvall(self, sock, count):
        # 바이트 문자열
        buf = b''

        while count:
            newbuf = sock.recv(count)

            if not newbuf: return None
            
            buf += newbuf
            count -= len(newbuf)

        return buf


    def run_server(self):
        print(f'Server Start with IP: {self.HOST}')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = (self.HOST, self.PORT) 
        self.server_socket.bind(server_address)
        self.server_socket.listen()

        print("Server is listening...")


        try:
            while True:
                client_socket, client_address = self.server_socket.accept()
                client_id = str(client_address[0])

                if client_address[0] in self.ip_name:
                    print(f'연결 수락됨: {self.ip_name[client_address[0]]}')

                else:
                    print("미등록 사용자입니다. : ", client_address[0])
                    
                print("client_id : ", client_id)

                self.client_sockets.append(client_socket)

                print(f'참여한 클라이언트 수: {len(self.client_fsockets)}')
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

            
                # 쓰레드 종료 플래그 생성 또는 재설정
                if client_id in self.thread_stop_flags:
                    self.thread_stop_flags[client_id].set()  # 기존 쓰레드 종료 신호

                    if client_id in self.threads:
                        self.threads[client_id].join()  # 종료될 때까지 기다리기

                self.thread_stop_flags[client_id] = threading.Event()

                # 클라이언트별 쓰레드 시작
                self.threads[client_id] = threading.Thread(
                                                            target=self.image_callback, 
                                                            args=(client_socket, client_id, self.thread_stop_flags[client_id])
                                                            )
                self.threads[client_id].start()

                
        except KeyboardInterrupt:
            print('서버를 종료합니다.')

            # 종료 시 모든 스레드 정리
            for flag in self.thread_stop_flags:
                flag.clear()

            for t in self.threads:
                t.join()

            for client_socket in self.client_sockets:
                client_socket.close()

            self.server_socket.close()
            
        
        finally:
            # 어떠한 경우에도 마지막에 서버 소켓을 닫음
            if self.server_socket:
                self.server_socket.close()



def main():
    # Initialize both nodes
    server_node = MyServer()

    # Run the server in a separate thread
    server_thread = threading.Thread(target=server_node.run_server)
    server_thread.start()

    # Shutdown and clean up
    server_thread.join()

if __name__ == '__main__':
    main()