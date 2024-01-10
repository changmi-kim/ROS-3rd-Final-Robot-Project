import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import serial
import time
import numpy as np
import math

ser = None

class ArduinoProtocolSender(Node):
    def __init__(self):
        super().__init__('arduino_protocol_sender')
        self.subscriber_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.subscriber_command = self.create_subscription(String, '/command', self.command_callback, 10)
        self.enable_motor = 0
        # self.subscriber_

    def cmd_vel_callback(self, twist):
        # enable = 1
        l_lamp = 100
        r_lamp = 100

        wheel_radius = 0.035
        wheel_separation_width = 0.175
        wheel_separation_length = 0.1

        wheel_l_vel_rad = (1 / wheel_radius) * (twist.linear.x - twist.linear.y - 
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        wheel_r_vel_rad = (1 / wheel_radius) * (twist.linear.x + twist.linear.y + 
                                            (wheel_separation_width + wheel_separation_length) * twist.angular.z)
        
        wheel_l_vel_rpm = wheel_l_vel_rad * 60 / (2 * math.pi)
        wheel_r_vel_rpm = wheel_r_vel_rad * 60 / (2 * math.pi)

        # 0~333rpm -> 0~1000pwm mapping
        wheel_l_vel_pwm = np.int16(wheel_l_vel_rpm * 1000 / 333)
        wheel_r_vel_pwm = np.int16(wheel_r_vel_rpm* 1000 / 333) * (-1)

        command = [0xfa, 0xfe, 2, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 0xfa, 0xfd]

        command[3] = self.enable_motor
        command[4] = (np.uint8)(wheel_l_vel_pwm >> 8)
        command[5] = 0
        command[6] = (np.uint8)(wheel_r_vel_pwm >> 8)          
        command[7] = 0
        command[8] = l_lamp
        command[9] = r_lamp
        command[12] = (np.uint8)(sum(command[2:12]))

        ser.write(bytes(command))

    def command_callback(self, msg):
        command_msg = msg.data.lower()

        if command_msg == 'stop':
            self.enable_motor = 0

        elif command_msg == 'go':
            self.enable_motor = 1

def main(args=None):
    global ser
    rclpy.init(args=args)

    ser = serial.Serial('/dev/ttyArduino', 1000000)
    time.sleep(3)
    print('아두이노 연결 성공')

    arduino_protocol_sender = ArduinoProtocolSender()

    rclpy.spin(arduino_protocol_sender)

    arduino_protocol_sender.destroy_node()
    ser.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()