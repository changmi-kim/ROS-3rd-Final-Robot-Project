import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import numpy as np

class UltrasonicDistSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_dist_subscliber')
        self.subscriber_dist = self.create_subscription(String, '/dist_data', self.dist_sub_timer_callback, 10)
        self.publisher_command = self.create_publisher(String, '/command', 10)
        
    def dist_sub_timer_callback(self, msg):
        data = msg.data
        self.get_logger().info('sub Distance: {0}cm'.format(data))

        command_msg = String()

        if int(data) <= 100:
            command_msg.data = 'STOP'
        else:
            command_msg.data = 'GO'

        self.publisher_command.publish(command_msg)
        print(command_msg)

def main(args=None):
    rclpy.init(args=args)

    ultrasonic_dist_subscliber = UltrasonicDistSubscriber()

    rclpy.spin(ultrasonic_dist_subscliber)

    ultrasonic_dist_subscliber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()