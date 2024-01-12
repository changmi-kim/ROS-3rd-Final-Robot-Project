import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time

class UltrasonicDistPublisher(Node):

    def __init__(self):
        super().__init__('ultrasonic_dist_publisher')
        self.publisher_dist = self.create_publisher(String, '/dist_data', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.dist_pub_timer_callback)

        self.declare_parameter('arduino_port', '/dev/ttyArduino')
        self.declare_parameter('baud_rate', 11520)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        self.ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(3)
        self.get_logger().info('아두이노 연결 성공')
    
    def dist_pub_timer_callback(self):
        msg = String()
        msg.data = self.ser.readline().decode()
        msg.data = msg.data.replace('\r', '').replace('\n', '')
        self.publisher_dist.publish(msg)
        self.get_logger().info('distance: {0}cm'.format(msg.data))

def main(args=None):
    rclpy.init(args=args)

    ultrasonic_dist_publisher = UltrasonicDistPublisher()

    rclpy.spin(ultrasonic_dist_publisher)

    ultrasonic_dist_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()