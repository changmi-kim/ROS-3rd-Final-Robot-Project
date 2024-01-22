import numpy as np
import cv2
import sys
import argparse
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from my_minibot_aruco_package import aruco_processing
from aruco_processing import ARUCO_DICT
from geometry_msgs.msg import Twist, Pose
from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel
from cv_bridge import CvBridge, CvBridgeError



class CmdAndPose(Node):
    def __init__(self, aruco_dict_type, k, d):
        super().__init__('pose_estimation_node')
        self.cmd_publisher = self.create_publisher(Twist, Pose, 'base_controller/cmd_vel_unstamped', 10)
        self.safe_distance = 0.4
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
        self.aruco_dict_type = aruco_dict_type
        self.k = k  #aruco_dict_type, k, d는 ArUco 마커 탐지 및 위치 추정에 사용되는 매개변수입니다
        self.d = d
        self.last_time = time.time()
        self.update_interval = 0.05 # 업데이트 간격 (초 단위) 이걸 사용하면 실시간성을 조정할 수 있다.
        self.x = 0
        self.y = 0
        self.z = 0
    
    



    def control_robot(self, x, y, angle):
        twist = Twist()
        # Angle에 따른 회전 속도 설정
        if angle < -0.1 or angle > 0.1:
            twist.angular.z = -angle
        else:
            twist.angular.z = 0
        # 중앙에 위치할 때만 전진
        if x != -1:  # -1은 마커가 감지되지 않았을 경우를 가정한 값입니다.
            twist.linear.x = 0.1
        else:
            twist.linear.x = 0
        # cmd_vel 토픽에 Twist 메시지 발행
        self.publisher.publish(twist)

    def image_callback(self, msg, pose_estimation):
        self.aruco_result = CmdAndPoseVel()
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)

        except CvBridgeError as e:
            print(e)
        current_time = time.time()
        if current_time - self.last_time > self.update_interval:
            np_arr = np.frombuffer(cv_image.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            output = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
            cv2.imshow('Estimated Pose', output)
            cv2.waitKey(1)
            self.last_time = current_time  # 마지막 업데이트 시간 갱신

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="/home/kang/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package/calibration_matrix.npy")
    ap.add_argument("-d", "--D_Coeff", required=True, help="/home/kang/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package/distortion_coefficients.npy")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)
    
    aruco_dict_type = ARUCO_DICT[args["type"]]
    calibration_matrix_path = args["K_Matrix"]
    distortion_coefficients_path = args["D_Coeff"]

    k = np.load(calibration_matrix_path)
    d = np.load(distortion_coefficients_path)

    # k = np.load(args["K_Matrix"])
    # d = np.load(args["D_Coeff"])
    # aruco_dict_type = ARUCO_DICT[args["type"]]

    rclpy.init(args=None)
    # node = PoseEstimationNode(aruco_dict_type, k, d)
    rclpy.spin(CmdAndPose)
    CmdAndPose.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()