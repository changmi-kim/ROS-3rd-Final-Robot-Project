# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data, QoSProfile
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy
# import math


# class VisionTracker(Node):
#     def __init__(self):
#         super().__init__('vision_tracker')
#         self.qos = QoSProfile(depth=10)
#         self.cv_bridge = CvBridge()

#         self.subscriber_img = self.create_subscription(Image, '/debug_image', self.img_callback, qos_profile_sensor_data)
#         self.subscriber_vision_detection_result = self.create_subscription(Twist, '/detection_result', self.detection_result_callback, 10)

#         self.publisher_cmd_vel = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
#         self.publisher_debug_image_tracker = self.create_publisher(Image, '/debug_image_tracker', 10)

#         self.i = 0
#         self.twist = Twist()
#         self.twist.linear.x = 0.3
#         self.bbox_cx = 0
#         self.bbox_cy = 0
#         self.delta = 0.0

#     def img_callback(self, msg: Image):
#         cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
#         h, w, d = cv_image.shape
#         cv2.circle(cv_image, (int(self.bbox_cx), int(self.bbox_cy)), 20, (0, 0, 255), -1)

#         err = self.bbox_cx - w / 2
#         self.delta = err
#         self.twist.angular.z = (-1) * self.delta / 100
#         self.get_logger().info(f'angular.z = {self.twist.angular.z}')

#         self.publisher_cmd_vel.publish(self.twist)
#         self.publisher_debug_image_tracker.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding = msg.encoding))

#         cv2.imshow('debug_img_tracker', cv_image)
#         cv2.waitKey(10)

#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             self.stop()

#     def detection_result_callback(self, msg):
#         for detection in msg.detection_result:
#             self.bbox_cx = detection.bbox_center_x
#             self.bbox_cy = detection.bbox_center_y

#     def publish_stop_cmd(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = 0.0
#         self.publisher_cmd_vel.publish(self.twist)
#         self.get_logger().info(f'Robot stopped.')

# def main(args=None):
#     rclpy.init(args=args)

#     vision_tracker = VisionTracker()

#     try:
#         rclpy.spin(vision_tracker)

#     except KeyboardInterrupt:
#         vision_tracker.stop()
#         vision_tracker.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import argparse
# import numpy as np
# from rclpy.node import Node
# from aruco_processing import ARUCO_DICT
# from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel
# import time
# import datetime
# import sys

# id_to_label = {
#     70: "00",
#     71: "01",
#     72: "02",
#     73: "3",
#     74: "4",
#     75: "5",
#     76: "6",
#     77: "7",
#     78: "8",
#     79: "9",
#     80: "10",
#     81: "11",
#     82: "12",
#     83: "A1",
#     84: "A2",
#     85: "A3",
#     86: "A4",
   
# }


# class VisionTracker(Node):
#     def __init__(self, aruco_dict_type, k, d):
#         super().__init__('vision_tracker')
#         self.cv_bridge = CvBridge()
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#         self.aruco_dict_type = aruco_dict_type
#         self.k = k
#         self.d = d
#         self.last_time = time.time()
#         self.update_interval = 0.05 # 업데이트 간격 (초 단위) 이걸 사용하면 실시간성을 조정할 수 있다.


#     def image_callback(self, msg):
#         # np_arr = np.frombuffer(msg.data, np.uint8)
#         # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         # cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#         # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         # corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

#         current_time = time.time()
#         if current_time - self.last_time > self.update_interval:
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#             output, x, y = self.pose_estimation(image_np, self.aruco_dict, self.k, self.d)
#             cv2.imshow('Estimated Pose', output)
#             cv2.waitKey(1)
#             self.control_robot(x, y)
#             self.last_time = current_time # 마지막 업데이트 시간 갱신

#         # if len(corners) > 0:
#         #     # 가장 큰 ArUco 마커를 찾기
#         #     largest_marker = max(corners, key=lambda c: cv2.contourArea(c))
#         #     x, y = np.mean(largest_marker[0], axis=0)
#         #     h, w = cv_image.shape[:2]

#         #     # 이미지 중심과 ArUco 마커 중심 사이의 오차 계산
#         #     error_x = x - w / 2
#         #     error_y = y - h / 2

#         #     # 로봇 제어를 위한 Twist 메시지 생성
#         #     twist = Twist()
#         #     twist.linear.x = -0.01 * error_y  # 전진 및 후진 속도
#         #     twist.angular.z = -0.003 * error_x  # 회전 속도
#         #     self.publisher.publish(twist)
#     def pose_estimation(self, frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
#         parameters = cv2.aruco.DetectorParameters_create()

#         corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)
#         print(f"Detected corners: {corners}")
#         print(f"Detected IDs: {ids}")
#         print(f"Rejected points: {rejected_img_points}")

#         # 현재 시간을 얻습니다
#         current_time = datetime.datetime.now()
#         # 시간을 문자열 형식으로 변환합니다 (예: '2024-01-18 15:45:30')
#         time_string = current_time.strftime("%Y-%m-%d %H:%M:%S")
#         text_size = cv2.getTextSize(time_string, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
#         text_x = (frame.shape[1] - text_size[0]) // 2 # 가로 중앙 정렬
#         text_y = text_size[1] + 10 # 상단 여백을 10 픽셀로 설정
#         text_z = (frame.shape[1] - text_size[0]) // 8
#         cv2.putText(frame, time_string, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
#         cv2.putText(frame, "M-1",(text_z, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2) # 좌측 상단 끝에 로봇 번호 추가

#         if len(corners) > 0:
#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#             # 텍스트의 세로 간격
#             line_height = 30
#             closest_markers = {0.022: None, 0.065: None} # 초기에는 각 marker_size에 대한 값으로 None을 할당하여 아직 가장 가까운 마커가 정해지지 않았음을 나타냅니다.
#             closest_distances = {0.022: float('inf'), 0.065: float('inf')} # 초기 거리 값으로 float('inf')를 사용합니다. float('inf')는 무한대를 의미하는 특수한 값으로, 아직 어떤 마커도 탐지되지 않았음을 나타냅니다. 즉, 시작할 때는 가장 가까운 마커의 거리가 무한대라고 가정합니다.
            
#             for i in range(len(ids)): #["0", "1", "2", "9", "10", "11", "12"]

#                 if ids[i][0] in [70, 71, 72, 79, 80, 81, 82, 83, 84, 85, 86 ]:
#                     marker_size = 0.022
#                     text_y_offset = line_height
#                     axis_length = 0.01
#                 else:
#                     marker_size = 0.065
#                     text_y_offset = 8 * line_height
#                     axis_length = 0.03
                    
#                 rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, matrix_coefficients, distortion_coefficients) #기본 0.022m ,테스트 0.027m, 0.065m
#                 x, y, z = tvec[0][0]
#                 distance_to_marker = z  #cv2.aruco.estimatePoseSingleMarkers 함수를 사용하여 각 마커의 3차원 위치(tvec)와 회전(rvec)을 추정합니다.

#                 if distance_to_marker < closest_distances[marker_size]:
#                         closest_distances[marker_size] = distance_to_marker
#                         closest_markers[marker_size] = (i, x, y, z, ids[i][0])

#                 print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker*100} CM")
#                 print(f"X: {x*100:.2f} CM, Y: {y*100:.2f} CM, Z: {z*100:.2f} CM") # 소수점 2번쨰 자리  

#                 # cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#                 cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, axis_length)
#             for marker_size, marker_info in closest_markers.items():
#                 if marker_info is not None:
#                     i, x, y, z, id = marker_info
#                     # corners 배열의 i번째 마커의 모든 코너의 평균 좌표를 계산하여 중심 좌표를 구함
#                     center_x = int(np.mean(corners[i][0][:, 0])) # 모든 모서리의 x 좌표를 선택
#                     center_y = int(np.mean(corners[i][0][:, 1])) # 모든 모서리의 y 좌표를 선택
#                     text_y_offset = 70 + 7 * line_height if marker_size == 0.065 else 70 + line_height

#                      # 화면에 표시할 텍스트
#                     if id in id_to_label:
#                         number = id_to_label[id]
#                         # 화면에 숫자 표시
#                         cv2.putText(frame, str(number), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 2) #RGB가 아니라 BGR로 되어 있다. (노랑색) frame: 텍스트를 그릴 이미지 또는 프레임입니다. 이 경우, frame은 ArUco 마커를 인식한 비디오 프레임
#                         distance_to_marker = z
#                         # 텍스트의 세로 간격
#                         line_height = 30

#                         # 이미지의 너비 구하기
#                         frame_width = frame.shape[1] -10

#                         # 텍스트 너비 구하기 (가장 긴 텍스트를 기준으로)
#                         text = f"X: {x*100:.2f} CM"
#                         text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]
#                         text_1 = f"Distance: {distance_to_marker*100:.2f} CM"
#                         text_width_1 = cv2.getTextSize(text_1, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]
#                         text_2 = f"Robot Location: {number}"
#                         text_width_2 = cv2.getTextSize(text_2, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]

#                         # 텍스트 시작 위치의 x 좌표 계산
#                         x_position = frame_width - text_width - 15 # 오른쪽 여백 10픽셀
#                         Y_position = frame_width - text_width_1 + 5 #- 15 # x_position - 113
#                         Z_position = frame_width - text_width_2 + 5#- 15 # Y_position
                        
#                         cv2.putText(frame, f"Robot Location: {number}", (Z_position, 70 + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2) # 노랑색
#                         cv2.putText(frame, f"Distance: {distance_to_marker*100:.2f} CM", (Y_position, 70 + line_height + text_y_offset),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 71, 0), 2) # 하늘
#                         if z*100 <= 3.3:
#                             cv2.putText(frame, f"{number} Connect!!", (10, 160 + 40 * (int(marker_size * 1000) - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # 빨간색
#                             # Done" 아래에 "System OK" 텍스트 추가
#                             cv2.putText(frame, "System OK", (10, 160 + 40 * (int(marker_size * 1000) - 21)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)  # 녹색
#                             # # "System OK" 아래에 "Charging..." 텍스트 추가
#                             cv2.putText(frame, f"{number} Charging...", (10, 160 + 40 * (int(marker_size * 1000) - 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # 빨간색
                        
#                         elif z*100 <= 10:
#                             cv2.putText(frame, f"{number} Docking.....", (10, 160 + 40 * (int(marker_size * 1000) - 22)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)  # 빨간색
                    
#                         if -0.4 <= x*100 <= 0.4:
#                         # X 값이 -0.4와 0.4 사이에 있을 경우 빨간색으로 표시
#                             cv2.putText(frame, f"OK! X: {x*100:.2f} CM", (x_position, 70 + 2 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
#                         else:
#                             # 그렇지 않은 경우 초록색으로 표시
#                             cv2.putText(frame, f"X: {x*100:.2f} CM", (x_position, 70 + 2 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

#                         if -0.2 <= y*100 <= 0.25:
#                         # Y 값이 -0.4와 0.4 사이에 있을 경우 빨간색으로 표시
#                             cv2.putText(frame, f"OK! Y: {y*100:.2f} CM", (x_position, 70 + 3 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
#                         else:
#                             # 그렇지 않은 경우 초록색으로 표시
#                             cv2.putText(frame, f"Y: {y*100:.2f} CM", (x_position, 70 + 3 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

#                         if z*100 <= 3.3:
#                             cv2.putText(frame, f"OK! Z: {z*100:.2f} CM", (x_position, 70 + 4 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
#                         else:
#                             cv2.putText(frame, f"Z: {z*100:.2f} CM", (x_position, 70 + 4 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
#                     axis_points_3d = np.float32([[axis_length, 0, 0], [0, axis_length, 0], [0, 0, axis_length]]).reshape(-1, 3)
                    
#                     # 3D 좌표를 2D 이미지 평면에 투영합니다.
#                     image_points, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, matrix_coefficients, distortion_coefficients)

#                     # 이미지에 X, Y, Z 축을 표시합니다.
#                     for point, axis_name in zip(image_points, ["X", "Y", "Z"]): #zip은 파이썬 내장 함수 중 하나로, 두 개 이상의 시퀀스(리스트, 튜플 등)를 인자로 받아 각 시퀀스에서 동일한 인덱스에 위치한 요소들을 묶어 새로운 튜플로 만들어주는 역할을 합니다. 이 때, 묶이는 요소의 개수는 인자로 전달된 시퀀스 중 가장 짧은 시퀀스의 길이에 맞추어집니다.
#                         x, y = point.ravel()
#                         if axis_name == "X":
#                             color = (0, 0, 255)  # 빨간색 (BGR)
#                         elif axis_name == "Y":
#                             color = (0, 255, 0)  # 초록색 (BGR)
#                         elif axis_name == "Z":
#                             color = (255, 0, 0)  # 파란색 (BGR)
#                         # 각 축에 대한 정보를 표시할 위치를 계산합니다.
            
#                         cv2.putText(frame, axis_name, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    
                  
        
#         return frame, x, y
    
#     def control_robot(self, x, y):
#         # 로봇 제어 로직
#         twist = Twist()
#         # 로봇 제어 알고리즘 구현
#         self.publisher.publish(twist)
#         self.x = x
#         self. y = y
#     def stop(self):
#         # 로봇을 멈추기 위한 Twist 메시지 생성
#         stop_twist = Twist()
#         stop_twist.linear.x = 0.0
#         stop_twist.angular.z = 0.0
#         self.publisher.publish(stop_twist)
#         self.get_logger().info('Stopping the robot')
# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("-k", "--K_Matrix", required=True, help="/home/kang/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package/calibration_matrix.npy")
#     ap.add_argument("-d", "--D_Coeff", required=True, help="/home/kang/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package/distortion_coefficients.npy")
#     ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
#     args = vars(ap.parse_args())

#     if ARUCO_DICT.get(args["type"], None) is None:
#         print(f"ArUCo tag type '{args['type']}' is not supported")
#         sys.exit(0)
    
#     aruco_dict_type = ARUCO_DICT[args["type"]]
#     calibration_matrix_path = args["K_Matrix"]
#     distortion_coefficients_path = args["D_Coeff"]

#     k = np.load(calibration_matrix_path)
#     d = np.load(distortion_coefficients_path)

#     # k = np.load(args["K_Matrix"])
#     # d = np.load(args["D_Coeff"])
#     # aruco_dict_type = ARUCO_DICT[args["type"]]

#     rclpy.init(args=None)
#     node = VisionTracker(aruco_dict_type, k, d)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# # def main(args=None):
# #     rclpy.init(args=args)
# #     vision_tracker = VisionTracker()
# #     rclpy.spin(vision_tracker)
# #     vision_tracker.destroy_node()
# #     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

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
        self.cv_bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
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


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import time
# import datetime
# from aruco_processing import ARUCO_DICT
# from my_minibot_aruco_package_msgs.msg import CmdAndPoseVel

# class VisionTracker(Node):
#     def __init__(self):
#         super().__init__('vision_tracker')
#         self.cv_bridge = CvBridge()
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.cv_bridge = CvBridge()
#         self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
#         self.aruco_params = cv2.aruco.DetectorParameters_create()

#     def image_callback(self, data):
#         try:
#             np_arr = np.frombuffer(data.data, np.uint8)
#             cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#             gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#             corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

#             if ids is not None:
#                 # Find the center of the ArUco marker
#                 x_center, y_center = np.mean(corners[0][0], axis=0)
#                 h, w = cv_image.shape[:2]

#                 # Calculate the error between the center of the image and the marker
#                 error_x = x_center - w / 2
#                 error_y = y_center - h / 2

#                 # Simple proportional controller
#                 twist = Twist()
#                 twist.linear.x = -0.01 * error_y
#                 twist.angular.z = -0.003 * error_x
#                 self.publisher.publish(twist)
#         except KeyboardInterrupt:
#             print("프로세스가 사용자에 의해 중단되었습니다.")
#             # 필요한 정리 작업 수행
#         except Exception as e:
#             print(f"예외 발생: {e}")
#             # 예외에 대한 처리
#             cv2.imshow("compressed_image_subscriber", cv_image)
#             cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)
#     node = VisionTracker()
#     rclpy.spin(node)
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()