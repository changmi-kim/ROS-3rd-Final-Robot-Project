import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from aruco_processing import ARUCO_DICT
from geometry_msgs.msg import PoseStamped
from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel

class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # 가장 첫 번째 ArUco 마커에 대해서만 작업
            first_marker = corners[0][0]
            x_center, y_center = np.mean(first_marker, axis=0)
            h, w = cv_image.shape[:2]

            # 이미지 중심과 ArUco 마커 중심 사이의 오차 계산
            error_x = x_center - w / 2
            error_y = y_center - h / 2

            # 로봇 제어를 위한 Twist 메시지 생성
            twist = Twist()
            twist.linear.x = min(max(-0.01 * error_y, -0.5), 0.5)  # 전진 및 후진 속도 제한
            twist.angular.z = min(max(-0.003 * error_x, -1.0), 1.0)  # 회전 속도 제한
            self.publisher.publish(twist)

            # 마커와 축을 이미지에 그리기
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.05, self.k, self.d)
            cv2.aruco.drawAxis(cv_image, self.k, self.d, rvec, tvec, 0.1)

        cv2.imshow("visionTrackor_image_subscriber", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()