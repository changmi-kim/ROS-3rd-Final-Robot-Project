import socket
import threading
import cv2
import numpy as np



class MyServer():

    def __init__(self):
        super().__init__()
        self.client_sockets = []
        self.client_sockets_lock = threading.Lock()
        self.HOST = "192.168.1.7" # 로컬 IP 주소
        self.PORT = 3306
        self.server_socket = None

        self.client_thread_count = 0
        self.image_thread_count = 0

        self.ip_name = {"192.168.1.14" : "minibot1",
                        "192.168.1.7" : "minibot2",
                        }
        
        self.images = {}  # 클라이언트별 최신 이미지를 저장하는 딕셔너리
        self.display_client = None  # 현재 표시중인 클라이언트

        self.thread_stop_flags = {}  # 쓰레드들 플레그 확인
        self.threads = {}  # 스레드 참조를 저장할 딕셔너리
                        
                

    def client_threaded(self, client_socket, client_address, flag):
        try:
            self.client_thread_count += 1
            print("생성된 고객 쓰레드 갯수 : ", self.client_thread_count)
            print(f'Connected by: {client_address[0]}:{client_address[1]}')
            threading.Thread(target=self.image_callback, args=(client_socket, flag)).start()

        except ConnectionResetError as e:
            print(f'Disconnected by {client_address[0]}:{client_address[1]}')

    
    def user_input_thread(self):
        while True:
            client_id = input("보여줄 클라이언트 번호를 입력하세요 (숨기려면 -1): ")
            if client_id in self.images or client_id == "-1":
                self.display_client = client_id
            else:
                print(f"잘못된 클라이언트 번호: {client_id}")

            
    def image_callback(self, client_socket, client_id, stop_event):
        self.image_thread_count += 1
        print("생성된 이미지 쓰레드 갯수 : ", self.image_thread_count)
        try:
            while not stop_event.is_set():
            
                # client에서 받은 stringData의 크기 (==(str(len(stringData))).encode().ljust(16))
                length = self.recvall(client_socket, 16)
                stringData = self.recvall(client_socket, int(length))
                data = np.fromstring(stringData, dtype = 'uint8')
                
                #data를 디코딩한다.
                frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

                self.images[client_id] = frame  # 클라이언트 별로 이미지 저장

                if self.display_client == client_id:
                    cv2.imshow('ImageWindow', frame)

                elif self.display_client == "-1":
                    cv2.destroyAllWindows()

                cv2.waitKey(1)

        except Exception as e:
            print(f'예외 발생: {e}')
                    

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
    

    def thread_start(self, thread_number):
        print(self.active_thread, thread_number)
        if self.active_thread is not None:
            self.run_flags[self.active_thread].clear()

        self.threads[thread_number].start()
        self.active_thread = thread_number
    

    def run_server(self):
        print(f'Server Start with IP: {self.HOST}')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = (self.HOST, self.PORT) 
        self.server_socket.bind(server_address)
        self.server_socket.listen()

        print("Server is listening...")

        input_thread = threading.Thread(target=self.user_input_thread)
        input_thread.start()


        try:
            while True:
                client_socket, client_address = self.server_socket.accept()
                client_id = str(client_address[0])
                print(f'연결 수락됨: {self.ip_name[client_address[0]]}')
                print("client_id : ", client_id)

                print(f'참여한 클라이언트 수: {len(self.client_sockets)}')
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

                with self.client_sockets_lock:
                    self.client_sockets.append(client_socket)

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
            for flag in self.run_flags:
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