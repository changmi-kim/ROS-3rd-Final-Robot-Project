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
                

    def client_threaded(self, client_socket, client_address):
        try:
            print(f'Connected by: {client_address[0]}:{client_address[1]}')
            threading.Thread(target=self.image_callback, args=(client_socket, )).start()

        except ConnectionResetError as e:
            print(f'Disconnected by {client_address[0]}:{client_address[1]}')
                

    
    def image_callback(self, client_socket):
        try:
            while True:
            
                # client에서 받은 stringData의 크기 (==(str(len(stringData))).encode().ljust(16))
                length = self.recvall(client_socket, 16)
                stringData = self.recvall(client_socket, int(length))
                data = np.fromstring(stringData, dtype = 'uint8')
                
                #data를 디코딩한다.
                frame = cv2.imdecode(data, cv2.IMREAD_COLOR)
                cv2.imshow('ImageWindow',frame)
                cv2.waitKey(1)
                    

        finally:  
            client_socket.close()

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
                print('Wait...')
                client_socket, client_address = self.server_socket.accept()
                print(f'연결 수락됨: {client_address}')

                with self.client_sockets_lock:
                    self.client_sockets.append(client_socket)
                    threading.Thread(target=self.client_threaded, args=(client_socket, client_address)).start()
                

                print(f'참여한 클라이언트 수: {len(self.client_sockets)}')
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())


        except KeyboardInterrupt:
            print('서버를 종료합니다.')
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