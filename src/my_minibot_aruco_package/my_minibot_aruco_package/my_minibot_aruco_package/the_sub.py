import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist
import subprocess
import time
import numpy as np
# from aruco_processing import ARUCO_DICT

class Subscriber(Node):
    
    def __init__(self):
        super().__init__('sub')
        self.subscription = self.create_subscription(
            Point,
            'test',     # project topic name : selfie
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        

        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
    
    # def callback(self, msg):
        
    #     self.get_logger().info(f"Received position: x={msg.x}, y={msg.y}, z={msg.z}")
            
        self.declare_parameter("some_threshold", 1.0)
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.5)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.3)
        self.declare_parameter("max_size_thresh", 0.3)
        self.declare_parameter("min_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        self.some_threshold = self.get_parameter('some_threshold')
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.min_size_thresh = self.get_parameter('min_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000


    def timer_callback(self):
        twist = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
            print(self.target_dist)
            if (self.target_dist < self.max_size_thresh):
                twist.linear.x = 0.0
            elif (self.target_dist > self.min_size_thresh):
                twist.linear.x = self.forward_chase_speed
            twist.angular.z = self.degree
            self.get_logger().info('msg: {} {}'.format(twist.linear.x, twist.angular.z))

        # else:
        #     # self.get_logger().info('Target lost')
        #     msg.angular.z = self.search_angular_speed
        self.publisher_.publish(twist)

    def listener_callback(self, msg):
        f = self.filter_value
        self.target_val = self.target_val * f + msg.x * (1-f)
        # self.get_logger().info('target_val: {}'.format(self.target_val))  # -0.08 ~ 0.08
        self.target_dist = self.target_dist * f + msg.z * (1-f)
        self.lastrcvtime = time.time()
        if msg.x != 0:
            self.degree = np.arcsin(msg.x / np.sqrt((msg.x)**2 + (msg.z)**2))
        else :
            self.degree = 0.0
        # self.get_logger().info('Received: {} {}'.format(msg.x, msg.y))


def main(args=None):
    rp.init(args=args)
    
    node = Subscriber()
    rp.spin(node)
    
    node.destroy_node() 
    rp.shutdown()

if __name__ == '__main__':
    main()
