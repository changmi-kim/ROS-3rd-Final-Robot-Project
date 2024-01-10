import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException  # 외부에서 실행이 중단 되었을 경우 발생하는 오류
from rclpy.qos import QoSProfile  # QoS 프로파일 호출
from rclpy.qos import QoSReliabilityPolicy  # QoS 신뢰성 설정
# from rclpy.qos import qos_profile_sensor_data  # 센서 데이터 같이 빠르게 변하는 데이터 처리
from communication_msgs.msg import Talker 



class ControllerTalker(Node):

    def __init__(self):
        super().__init__('Main_PC_talker')
        qos_profile = QoSProfile(depth=10, 
                                 reliability=QoSReliabilityPolicy.RELIABLE,
                                )
        self.pub = self.create_publisher(Talker, 'command', qos_profile)
        self.get_logger().info('Chatting has started. Press Enter to send a message.')
        self.input_thread_func()

    def input_thread_func(self):
        while True:
            self.publish_message()

    def publish_message(self):
        msg = Talker()
        msg.talker_name = "Minibot_talker"
        msg.message = input('Enter your message: ')
        self.pub.publish(msg)
        self.get_logger().info(f'Published: "{msg.message}"')



def main(args=None):
    rclpy.init(args=args)
    talker = ControllerTalker()

    try:
        rclpy.spin(talker)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        talker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()