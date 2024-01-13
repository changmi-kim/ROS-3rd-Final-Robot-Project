# import numpy as np
# import cv2
# import sys
# import argparse
# import rclpy
# import time
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from aruco_processing import ARUCO_DICT

# def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
#     parameters = cv2.aruco.DetectorParameters_create()

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     print(f"Detected corners: {corners}")
#     print(f"Detected IDs: {ids}")
#     print(f"Rejected points: {rejected_img_points}")

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     if len(corners) > 0:
#         for i in range(len(ids)):
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients)
#             x, y, z = tvec[0][0]
#             distance_to_marker = z

#             print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker} meters")
#             print(f"X: {x}, Y: {y}, Z: {z}")

#             cv2.putText(frame, f"Distance: {distance_to_marker:.2f} meters", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             cv2.putText(frame, f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#             cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

#     return frame

# class PoseEstimationNode(Node):
#     def __init__(self, aruco_dict_type, k, d):
#         super().__init__('pose_estimation_node')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)
#         self.aruco_dict_type = aruco_dict_type
#         self.k = k
#         self.d = d

#     def image_callback(self, data):
#         np_arr = np.frombuffer(data.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         output = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
#         cv2.imshow('Estimated Pose', output)
#         cv2.waitKey(1)

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
#     node = PoseEstimationNode(aruco_dict_type, k, d)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

    # (aruco) kang@kang-pc:~/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package$ 
    # minibot_aruco 쳐서 bashrc 활성화 시키기
    # source ~/venv/aruco/bin/activate   로 (aruco) 가상환경 활성화 시켜준다.
    # python pose_estimation_test2.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
    
# import numpy as np
# import cv2
# import sys
# import argparse
# import rclpy
# import time
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from aruco_processing import ARUCO_DICT

# def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
#     parameters = cv2.aruco.DetectorParameters_create()

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     print(f"Detected corners: {corners}")
#     print(f"Detected IDs: {ids}")
#     print(f"Rejected points: {rejected_img_points}")

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     if len(corners) > 0:
#         for i in range(len(ids)):
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients)
#             x, y, z = tvec[0][0]
#             distance_to_marker = z

#             print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker} meters")
#             print(f"X: {x}, Y: {y}, Z: {z}")

#             cv2.putText(frame, f"Distance: {distance_to_marker:.2f} meters", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#             cv2.putText(frame, f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#             cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

#     return frame

# class PoseEstimationNode(Node):
#     def __init__(self, aruco_dict_type, k, d):
#         super().__init__('pose_estimation_node')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)
#         self.aruco_dict_type = aruco_dict_type
#         self.k = k
#         self.d = d
#         self.last_time = time.time()
#         self.update_interval = 0.08 # 업데이트 간격 (초 단위) 이걸 사용하면 실시간성을 조정할 수 있다.

#     def image_callback(self, data):
#         current_time = time.time()
#         if current_time - self.last_time > self.update_interval:
#             np_arr = np.frombuffer(data.data, np.uint8)
#             image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#             output = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
#             cv2.imshow('Estimated Pose', output)
#             cv2.waitKey(1)
#             cv2.waitKey(1)
#             self.last_time = current_time  # 마지막 업데이트 시간 갱신

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
#     node = PoseEstimationNode(aruco_dict_type, k, d)
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#     #(aruco) kang@kang-pc:~/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package$ 
#     #minibot_aruco 쳐서 bashrc 활성화 시키기
#     #source ~/venv/aruco/bin/activate   로 (aruco) 가상환경 활성화 시켜준다.
#     #python pose_estimation_test2.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100

import numpy as np
import cv2
import sys
import argparse
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from aruco_processing import ARUCO_DICT

# ID를 숫자와 문자로 매핑하는 사전 정의
id_to_label = {
    70: "0",
    71: "1",
    72: "2",
    73: "3",
    74: "4",
    75: "5",
    76: "6",
    77: "7",
    78: "8",
    79: "9",
    80: "10",
    81: "11",
    82: "12",
    83: "A1",
    84: "A2",
    85: "A3",
    86: "A4",
   
}

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

    print(f"Detected corners: {corners}")
    print(f"Detected IDs: {ids}")
    print(f"Rejected points: {rejected_img_points}")

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

    # if len(corners) > 0:
    #     cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    #     for i in range(len(ids)):
    #         rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients) #기본 0.022m ,테스트 0.027m, 0.065m
    #         x, y, z = tvec[0][0]
    #         distance_to_marker = z

    #         print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker*100} CM")
    #         print(f"X: {x}, Y: {y}, Z: {z}")

    #         cv2.putText(frame, f"Distance: {distance_to_marker*100:.2f} CM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #         cv2.putText(frame, f"X: {x*100:.2f}, Y: {y*100:.2f}, Z: {z*100:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    #         cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    #         cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
    #         # ID가 매핑 사전에 있는 경우 숫자, 문자를 표시
    #         if ids[i][0] in id_to_label:
    #             number = id_to_label[ids[i][0]]
    #             # 숫자를 표시할 위치 (마커 중심)
    #             center_x = int(sum([corner[0] for corner in corners[i][0]]) / 4)
    #             center_y = int(sum([corner[1] for corner in corners[i][0]]) / 4)
    #             # 화면에 숫자 표시
    #             cv2.putText(frame, str(number), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 2) #RGB가 아니라 BGR로 되어 있다.
    #return frame

    if len(corners) > 0:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
         # 텍스트의 세로 간격
        line_height = 30
        marker_size = 0.022
        text_y_offset = line_height

        for i in range(len(ids)):   #["0", "1", "2", "9", "10", "11", "12"]

            if ids[i][0] in [70, 71, 72, 79, 80, 81, 82, 83, 84, 85, 86 ]:
                marker_size = 0.022
                text_y_offset = line_height
            else:
                marker_size = 0.065
                text_y_offset = 8 * line_height
                
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, matrix_coefficients, distortion_coefficients) #기본 0.022m ,테스트 0.027m, 0.065m
            x, y, z = tvec[0][0]
            distance_to_marker = z

            print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker*100} CM")
            print(f"X: {x}, Y: {y}, Z: {z}")

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            # ID가 매핑 사전에 있는 경우 숫자, 문자를 표시
            if ids[i][0] in id_to_label:
                number = id_to_label[ids[i][0]]
    
                # corners 배열의 i번째 마커의 모든 코너의 평균 좌표를 계산하여 중심 좌표를 구함
                center_x = int(np.mean(corners[i][0][:, 0])) # 모든 모서리의 x 좌표를 선택
                center_y = int(np.mean(corners[i][0][:, 1])) # 모든 모서리의 y 좌표를 선택

                # 화면에 숫자 표시
                cv2.putText(frame, str(number), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 2) #RGB가 아니라 BGR로 되어 있다. 노랑색

                # cv2.putText(frame, f"LOC: {number}, Distance: {distance_to_marker*100:.2f} CM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)     #기존의 텍스트 표시방식                           
                # cv2.putText(frame, f"X: {x*100:.2f}, Y: {y*100:.2f}, Z: {z*100:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # 텍스트의 세로 간격
                line_height = 30

                # 이미지의 너비 구하기
                frame_width = frame.shape[1] -10

                # 텍스트 너비 구하기 (가장 긴 텍스트를 기준으로)
                text = f"X: {x*100:.2f} CM"
                text_width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]
                text_1 = f"Distance: {distance_to_marker*100:.2f} CM"
                text_width_1 = cv2.getTextSize(text_1, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]
                text_2 = f"Robot Location: {number}"
                text_width_2 = cv2.getTextSize(text_2, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0]

                # 텍스트 시작 위치의 x 좌표 계산
                x_position = frame_width - text_width - 15 # 오른쪽 여백 10픽셀
                Y_position = frame_width - text_width_1 - 15 # x_position - 113
                Z_position = frame_width - text_width_2 - 15 # Y_position
                
                # aruco mrker 숫자 표시 및 거리 측정
                cv2.putText(frame, f"Robot Location: {number}", (Z_position, 70 + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2) # 노랑색
                cv2.putText(frame, f"Distance: {distance_to_marker*100:.2f} CM", (Y_position, 70 + line_height + text_y_offset),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 71, 0), 2) # 하늘

                # X, Y, Z 값을 이미지에 그리기
                cv2.putText(frame, f"X: {x*100:.2f} CM", (x_position, 70 + 2 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2) #초록
                cv2.putText(frame, f"Y: {y*100:.2f} CM", (x_position, 70 + 3 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(frame, f"Z: {z*100:.2f} CM", (x_position, 70 + 4 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            else:
                marker_size = 0.065
                text_y_offset = 6 * line_height

    
                # cv2.putText(frame, f"Robot Location: {number}", (Z_position, 70 + 6 * line_height + text_y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2) # 노랑색
                # cv2.putText(frame, f"Distance: {distance_to_marker*100:.2f} CM", (Y_position, 70 + 7 * line_height + text_y_offset),cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 71, 0), 2) # 파랑
            

       

    return frame

class PoseEstimationNode(Node):
    def __init__(self, aruco_dict_type, k, d):
        super().__init__('pose_estimation_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
        self.aruco_dict_type = aruco_dict_type
        self.k = k
        self.d = d
        self.last_time = time.time()
        self.update_interval = 0.05 # 업데이트 간격 (초 단위) 이걸 사용하면 실시간성을 조정할 수 있다.

    def image_callback(self, data):
        current_time = time.time()
        if current_time - self.last_time > self.update_interval:
            np_arr = np.frombuffer(data.data, np.uint8)
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
    node = PoseEstimationNode(aruco_dict_type, k, d)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


    #(aruco) kang@kang-pc:~/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package$ 
    #minibot_aruco 쳐서 bashrc 활성화 시키기
    #source ~/venv/aruco/bin/activate   로 (aruco) 가상환경 활성화 시켜준다.
    #python pose_estimation_test2.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100