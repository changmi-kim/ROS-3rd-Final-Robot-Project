import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time



class AriduinoConnect(Node):

    def __init__(self):
        super().__init__('connect_arduino')

        self.declare_parameter('arduino_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 11520)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(3)
        self.get_logger().info("아두이노 연결 성공!")

        self.call_my_name

    
    def call_my_name(self):
        self.get_logger().info("연결되었어 연결되었어")


def main(args=None):
    rclpy.init(args=args)
    arduino = AriduinoConnect()
    rclpy.spin(arduino)

    arduino.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


        