import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, Duration
from cv_bridge import CvBridge


class MyClient(Node):

    def __init__(self):
        super().__init__('minibot_cam_publish')

        self.qos_profile_ = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        history=HistoryPolicy.KEEP_LAST,
                        deadline=Duration(seconds=0, nanoseconds=100000000),  # 예: 100ms의 데드라인
                        )

        self.image_publisher_ = self.create_publisher(Image, 'minibot2_image', self.qos_profile_)
        self.bridge_ = CvBridge()
        self.cap_ = cv2.VideoCapture(0) 


        self.declare_parameter('width', 640)
        self.declare_parameter('length', 480)

        self.width = self.get_parameter('width').value
        self.length = self.get_parameter('length').value

        self.time_preiod_ = 0.01
        self.timer = self.create_timer(self.time_preiod_, self.camera_callback)


    def camera_callback(self):
        while rclpy.ok():
            ret, frame = self.cap_.read()
            
            frame = cv2.resize(frame, (self.width, self.length))

            if ret:
                image = self.bridge_.cv2_to_imgmsg(frame, 'bgr8')
                self.image_publisher_.publish(image)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    client_node = MyClient()
    rclpy.spin(client_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
