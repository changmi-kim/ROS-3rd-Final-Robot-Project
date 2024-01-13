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
    # python pose_estimation_test1.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100
    
import numpy as np
import cv2
import sys
import argparse
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from aruco_processing import ARUCO_DICT

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

    print(f"Detected corners: {corners}")
    print(f"Detected IDs: {ids}")
    print(f"Rejected points: {rejected_img_points}")

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients)
            x, y, z = tvec[0][0]
            distance_to_marker = z

            print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker} meters")
            print(f"X: {x}, Y: {y}, Z: {z}")

            cv2.putText(frame, f"Distance: {distance_to_marker:.2f} meters", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"X: {x:.2f}, Y: {y:.2f}, Z: {z:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)

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
        self.update_interval = 0.08 # 업데이트 간격 (초 단위) 이걸 사용하면 실시간성을 조정할 수 있다.

    def image_callback(self, data):
        current_time = time.time()
        if current_time - self.last_time > self.update_interval:
            np_arr = np.frombuffer(data.data, np.uint8)
            image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            output = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
            cv2.imshow('Estimated Pose', output)
            cv2.waitKey(1)
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
    #python pose_estimation_test1.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100


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
#     if frame is None:
#         return None, None
    
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

#     return frame, (corners, ids, rejected_img_points)

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
#         self.last_printed_data = None
#         self.last_print_time = time.time()

        

#     # def image_callback(self, data):
        
#     #     np_arr = np.frombuffer(data.data, np.uint8)
#     #     image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#     #     if image_np is not None:
#     #         output, data = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
#     #         if output is not None:  # output이 유효한 이미지인지 확인
#     #             cv2.imshow('Estimated Pose', output)
#     #             cv2.waitKey(1)
            
#     #         self.last_data = data

#     #         current_time = time.time()
#     #         if current_time - self.last_print_time >= 0.5:
#     #             if not np.array_equal(self.last_data, self.last_printed_data):
#     #                 self.print_data(self.last_data)
#     #                 self.last_print_time = current_time
#     #                 self.last_printed_data = self.last_data
        
#     def image_callback(self, data):
#             np_arr = np.frombuffer(data.data, np.uint8)
#             image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#             if image_np is not None:
#                 output, data = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
#                 if output is not None:
#                     cv2.imshow('Estimated Pose', output)
#                     cv2.waitKey(1)

#                 self.last_data = data
#                 current_time = time.time()

#                 if current_time - self.last_print_time >= 0.5:
#                     if self.last_data is not None and (self.last_printed_data is None or not np.array_equal(self.last_data[1], self.last_printed_data[1])):
#                         self.print_data(self.last_data)
#                         self.last_print_time = current_time
#                         self.last_printed_data = self.last_data

#     def print_data(self, data):
#         if data is not None:
#             corners, ids, rejected_img_points = data
#             if ids is not None:
#                 print(f"Detected IDs: {ids}")

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
    

# import numpy as np
# import cv2
# import sys
# import argparse
# import rclpy
# import time
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from aruco_processing import ARUCO_DICT

# # ID를 숫자와 문자로 매핑하는 사전 정의
# id_to_label = {
#     70: "0",
#     71: "1",
#     72: "2",
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

# def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
#     parameters = cv2.aruco.DetectorParameters_create()

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     print(f"Detected corners: {corners}")
#     print(f"Detected IDs: {ids}")
#     print(f"Rejected points: {rejected_img_points}")

#     corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

#     # if len(corners) > 0:
#     #     cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#     #     for i in range(len(ids)):
#     #         rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients) #기본 0.022m ,테스트 0.027m, 0.065m
#     #         x, y, z = tvec[0][0]
#     #         distance_to_marker = z

#     #         print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker*100} CM")
#     #         print(f"X: {x}, Y: {y}, Z: {z}")

#     #         cv2.putText(frame, f"Distance: {distance_to_marker*100:.2f} CM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#     #         cv2.putText(frame, f"X: {x*100:.2f}, Y: {y*100:.2f}, Z: {z*100:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#     #         cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#     #         cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
#     #         # ID가 매핑 사전에 있는 경우 숫자, 문자를 표시
#     #         if ids[i][0] in id_to_label:
#     #             number = id_to_label[ids[i][0]]
#     #             # 숫자를 표시할 위치 (마커 중심)
#     #             center_x = int(sum([corner[0] for corner in corners[i][0]]) / 4)
#     #             center_y = int(sum([corner[1] for corner in corners[i][0]]) / 4)
#     #             # 화면에 숫자 표시
#     #             cv2.putText(frame, str(number), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 2) #RGB가 아니라 BGR로 되어 있다.
#     # return frame
    
#     if len(corners) > 0:
#         cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#         for i in range(len(ids)):
#             rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients) #기본 0.022m ,테스트 0.027m, 0.065m
#             x, y, z = tvec[0][0]
#             distance_to_marker = z

#             print(f"Distance to ArUco marker {ids[i]}: {distance_to_marker*100} CM")
#             print(f"X: {x}, Y: {y}, Z: {z}")

#             cv2.aruco.drawDetectedMarkers(frame, corners, ids)
#             cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
#             # ID가 매핑 사전에 있는 경우 숫자, 문자를 표시
#             if ids[i][0] in id_to_label:
#                 number = id_to_label[ids[i][0]]
#                 # 숫자를 표시할 위치 (마커 중심)
#                 center_x = int(sum([corner[0] for corner in corners[i][0]]) / 4)
#                 center_y = int(sum([corner[1] for corner in corners[i][0]]) / 4)
#                 # 화면에 숫자 표시
#                 cv2.putText(frame, str(number), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 255), 2) #RGB가 아니라 BGR로 되어 있다.

#                 cv2.putText(frame, f"LOC {number} Distance: {distance_to_marker*100:.2f} CM", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
#                 cv2.putText(frame, f"X: {x*100:.2f}, Y: {y*100:.2f}, Z: {z*100:.2f}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
#     #python pose_estimation_test1.py --K_Matrix calibration_matrix.npy --D_Coeff distortion_coefficients.npy --type DICT_5X5_100