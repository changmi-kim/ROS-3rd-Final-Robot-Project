import rclpy
import socket
import threading
from rclpy.node import Node
from std_msgs.msg import String

class MyServer(Node):

    def __init__(self):
        super().__init__('my_server')
        self.client_sockets = []
        self.client_sockets_lock = threading.Lock()
        self.HOST = socket.gethostbyname(socket.gethostname())  # 로컬 IP 주소
        self.PORT = 3306
        self.server_socket = None

    def threaded(self, client_socket, client_address):
        self.get_logger().info(f'Connected by: {client_address[0]}:{client_address[1]}')

        while True:
            try:
                data = client_socket.recv(1024)
                if not data:
                    self.get_logger().info(f'Disconnected by {client_address[0]}:{client_address[1]}')
                    break

                self.get_logger().info(f'클라이언트로부터 받은 메시지: {data.decode()}')

                # 클라이언트의 메시지를 다른 클라이언트에게도 전송
                # for client in self.client_sockets:
                #     if client is not client_socket:
                #         client.send(data)

            except ConnectionResetError as e:
                self.get_logger().info(f'Disconnected by {client_address[0]}:{client_address[1]}')
                break

        with self.client_sockets_lock:
            if client_socket in self.client_sockets:
                self.client_sockets.remove(client_socket)
                self.get_logger().info(f'남은 클라이언트 수: {len(self.client_sockets)}')

        client_socket.close()

    def run_server(self):
        self.get_logger().info(f'Server Start with IP: {self.HOST}')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = (self.HOST, self.PORT)
        self.server_socket.bind(server_address)
        self.server_socket.listen()

        try:
            while True:
                self.get_logger().info('Wait...')
                client_socket, client_address = self.server_socket.accept()
                self.get_logger().info(f'연결 수락됨: {client_address}')

                with self.client_sockets_lock:
                    self.client_sockets.append(client_socket)
                    threading.Thread(target=self.threaded, args=(client_socket, client_address)).start()

                self.get_logger().info(f'참여한 클라이언트 수: {len(self.client_sockets)}')
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

        except KeyboardInterrupt:
            self.get_logger().info('서버를 종료합니다.')
            for client_socket in self.client_sockets:
                client_socket.close()
            self.server_socket.close()

def main(args=None):
    rclpy.init(args=args)
    server_node = MyServer()
    server_node.run_server()
    rclpy.spin(server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()