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
        while True:
            try:
                # 이미지 길이 정보 수신
                self.read_image_length(client_socket)
                

            finally:  
                client_socket.close()

                with self.client_sockets_lock:
                    if client_socket in self.client_sockets:
                        self.client_sockets.remove(client_socket)

                print(f'남은 클라이언트 수: {len(self.client_sockets)}')


    def recvall(self, sock, count):
        buf = bytearray(b'')

        while count:
            newbuf = sock.recv(min(1024, count))

            if not newbuf:
                return None
            
            buf += newbuf
            count -= len(newbuf)

        return buf
                
                
    # 청크용
    # def recvall(self, sock, length):
    #     data = bytearray()
    #     while len(data) < length:
    #         packet = sock.recv(length - len(data))
    #         if not packet:
    #             return None
    #         data.extend(packet)
    #     return data

    

    def read_image_length(self, sock):
        length_str = bytearray(b'')  # 바이트 문자열로 초기화

        while True:
            try:
                char = sock.recv(1)

                if not char:  # 비어 있는 바이트 문자열 확인 (소켓 닫힘)
                    raise ConnectionError("Socket connection closed while reading length")

                if char == b'\n':  # 바이트 형태로 개행 문자를 확인
                    image_length = int(length_str.decode('utf-8'))
                    print(image_length)
                    # 이미지 데이터 수신
                    image_data = self.recvall(sock, image_length)
                    print(image_data)

                    # 이미지 데이터를 numpy 배열로 변환 및 디코딩
                    image = np.frombuffer(image_data, np.uint8)
                    image = cv2.imdecode(image, cv2.IMREAD_COLOR)

                    # 이미지 처리 (예: 표시 또는 저장)
                    cv2.imshow("Received Image", image)
                    if cv2.waitKey(1) == ord('q'):
                        break

                length_str += char
            
            except:
                pass
                # print(f"Received char: {char}, Current length_str: {length_str}") 
            

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