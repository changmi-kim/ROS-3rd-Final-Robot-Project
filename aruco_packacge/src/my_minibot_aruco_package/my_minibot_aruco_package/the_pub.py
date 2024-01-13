import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Publisher(Node):

    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(String, 'test', 10)
        timer_period = 0.5 
        self.timer = self.create_timer(timer_period, self.callback)
        
    def callback(self):
        msg = String()  
        msg.data = "topic data"  
        self.publisher_.publish(msg) 
        print(msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = Publisher()

    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
