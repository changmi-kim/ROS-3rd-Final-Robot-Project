import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class Talker(Node):

    def __init__(self):
        super().__init__('Main_PC')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.get_logger().info('Chatting has started. Press Enter to send a message.')
        self.input_thread_func()

    def input_thread_func(self):
        while True:
            self.publish_message()

    def publish_message(self):
        msg = String()
        msg.data = input('Enter your message: ')
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()

    try:
        rclpy.spin(talker)
    except KeyboardInterrupt:
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()