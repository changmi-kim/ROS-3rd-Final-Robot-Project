import rclpy
import socket
from rclpy.node import Node
from std_msgs.msg import String

class MyClient(Node):

    def __init__(self):
        super().__init__('my_client')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_address = ('192.168.1.7', 3306)
        self.connect_to_server()

        self.create_subscription(String, 'server_response', self.recv_data, 10)
        self.publisher_ = self.create_publisher(String, 'client_input', 10)

    def connect_to_server(self):
        try:
            self.client_socket.connect(self.server_address)
            self.get_logger().info('Connected to the server')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to the server: {str(e)}')
            self.client_socket.close()
            rclpy.shutdown()
            exit(1)

    def recv_data(self, msg):
        response = msg.data
        self.get_logger().info(f'Server response: {response}')

    def run(self):
        while True:
            message = input("Enter a message: ")
            if message == "quit":
                self.client_socket.close()
                rclpy.shutdown()
                exit(0)
            self.client_socket.send(message.encode())
            self.get_logger().info(f'Sent message to server: {message}')


def main(args=None):
    rclpy.init(args=args)
    client_node = MyClient()
    client_node.run()
    rclpy.spin(client_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
