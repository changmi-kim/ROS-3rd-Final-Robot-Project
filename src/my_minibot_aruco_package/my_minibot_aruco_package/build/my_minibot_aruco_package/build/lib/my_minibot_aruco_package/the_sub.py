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
            10) #create_subscription 메소드를 사용해 Point 타입의 메시지를 'test' 토픽에서 구독하도록 설정합니다. 메시지가 도착하면 self.listener_callback 함수가 호출됩니다. 10은 메시지 큐의 크기를 의미합니다.
        self.subscription  # prevent unused variable warning self.subscription 변수를 참조하여 "사용되지 않은 변수"에 대한 경고를 방지합니다. 이는 일종의 코딩 컨벤션으로, 변수가 선언된 후 실제로 사용되지 않았을 때 발생할 수 있는 경고를 막기 위한 것입니다.
        

        self.publisher_ = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        #Twist 메시지 타입을 사용하는 퍼블리셔를 생성하고, 이를 self.publisher_에 할당합니다. 이 퍼블리셔는 /base_controller/cmd_vel_unstamped 토픽으로 메시지를 발행할 수 있으며, 여기서도 메시지 큐의 크기는 10으로 설정됩니다.
    # def callback(self, msg):
        
    #     self.get_logger().info(f"Received position: x={msg.x}, y={msg.y}, z={msg.z}")
            
        self.declare_parameter("some_threshold", 1.0)
        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.5)
        self.declare_parameter("forward_chase_speed", 0.05)
        self.declare_parameter("search_angular_speed", 0.3)
        self.declare_parameter("max_size_thresh", 0.3)
        self.declare_parameter("min_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        self.some_threshold = self.get_parameter('some_threshold')
        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value #get_parameter_value().double_value는 가져온 파라미터 값을 실수형(double)으로 변환하여 저장합니다. 이와 같은 방식으로 다른 파라미터들의 값을 실수형으로 변환하여 각각의 클래스 변수에 저장합니다.
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
            print(f"{self.target_dist:.2f}")
            if (self.target_dist < 4): #self.max_size_thresh
                twist.linear.x = 0.0
            else:
            # elif (self.target_dist > self.min_size_thresh):
                twist.linear.x = self.forward_chase_speed
            twist.angular.z = - self.degree / 3
            self.get_logger().info('msg: {:.2f} {:.2f} {:.2f}'.format(self.target_dist, twist.linear.x, twist.angular.z))
           
    # def timer_callback(self):
    #     twist = Twist()
    #     if (time.time() - self.lastrcvtime < self.rcv_timeout_secs):
    #         print(self.target_dist)
    #         if (self.target_dist < 4): #self.max_size_thresh
    #             twist.linear.x = 0.0
    #         else:
    #         # elif (self.target_dist > self.min_size_thresh):
    #             twist.linear.x = self.forward_chase_speed
    #         twist.angular.z = - self.degree / 2
    #         self.get_logger().info('msg: {} {}'.format(self.target_dist,twist.linear.x, twist.angular.z))

    #     # else:
    #     #     # self.get_logger().info('Target lost') 안씀
    #     #     msg.angular.z = self.search_angular_speed 안씀
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


#(aruco) kang@kang-pc:~/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package$ 
#ros2 run my_minibot_aruco_package the_sub

