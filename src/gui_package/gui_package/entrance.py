import cv2
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException  # 외부에서 실행이 중단 되었을 경우 발생하는 오류
from rclpy.qos import QoSProfile  # QoS 프로파일 호출
from rclpy.qos import QoSReliabilityPolicy  # QoS 신뢰성 설정
# from rclpy.qos import qos_profile_sensor_data  # 센서 데이터 같이 빠르게 변하는 데이터 처리
from communication_msgs.msg import Talker 

class ControllerListener(Node):

    def __init__(self):
        super().__init__('Main_PC_listener')
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(Talker, 'minibot_status', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg.talker_name}: {msg.message}")

def main(args=None):
    rclpy.init(args=args)
    listener = ControllerListener()

    try:
        rclpy.spin(listener)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()