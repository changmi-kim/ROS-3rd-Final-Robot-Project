import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
    def image_callback(self, data):
        # 압축 해제하여 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # cv2.IMREAD_COLOR는 이미지를 컬러로 읽기 위한 플래그입니다.
        # OpenCV를 사용하여 이미지를 표시
        cv2.imshow("compressed_image_subscriber", image_np)
        cv2.waitKey(1)  # OpenCV 창이 바로 닫히지 않도록 1밀리초 대기
def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageSubscriber()
    rclpy.spin(node)
    # 프로그램 종료 시 OpenCV 창을 정리
    cv2.destroyAllWindows()
    rclpy.shutdown()
if __name__ == '__main__':
    main()


# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = CompressedImageSubscriber()
#     rclpy.spin(image_subscriber)
#     image_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage

# class CompressedImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('compressed_image_subscriber')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)

#         # Load camera calibration data
#         self.calibration_matrix = np.load("calibration_matrix.npy")
#         self.distortion_coefficients = np.load("distortion_coefficients.npy")

#         # ArUco marker dictionary
#         self.aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
#         self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)

#     def image_callback(self, data):
#         # 압축 해제하여 이미지 데이터를 numpy 배열로 변환
#         np_arr = np.frombuffer(data.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # 카메라 캘리브레이션 적용
#         image_np = cv2.undistort(image_np, self.calibration_matrix, self.distortion_coefficients)

#         # ArUco 마커 감지
#         gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
#         aruco_params = cv2.aruco.DetectorParameters_create()
#         corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=aruco_params)

#         if ids is not None and len(ids) > 0:
#             for i in range(len(ids)):
#                 # ArUco 마커의 포즈(위치 및 방향) 추정
#                 rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.08, self.calibration_matrix, self.distortion_coefficients)

#                 # 표시할 문자열 생성
#                 pose_info = f"ID: {ids[i]}, X: {tvec[0][0]:.2f}, Y: {tvec[0][1]:.2f}, Z: {tvec[0][2]:.2f}"

#                 # 이미지에 표시
#                 cv2.aruco.drawAxis(image_np, self.calibration_matrix, self.distortion_coefficients, rvec, tvec, 0.1)
#                 cv2.putText(image_np, pose_info, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # OpenCV를 사용하여 이미지를 표시
#         cv2.imshow("Compressed Image with ArUco", image_np)
#         cv2.waitKey(1)  # OpenCV 창이 바로 닫히지 않도록 1밀리초 대기

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = CompressedImageSubscriber()
#     rclpy.spin(image_subscriber)
#     # 프로그램 종료 시 OpenCV 창을 정리
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import numpy as np
# import cv2
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# import os
# import argparse

# class CompressedImageSubscriber(Node):
#     def __init__(self):
#         super().__init__('compressed_image_subscriber')
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)

#         # Load camera calibration data
#         self.calibration_matrix = np.load("calibration_matrix.npy")
#         self.distortion_coefficients = np.load("distortion_coefficients.npy")

#         # ArUco marker dictionary
#         self.aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL
#         self.aruco_dict = cv2.aruco.Dictionary_get(self.aruco_dict_type)

#     def calibrate(self, dirpath, square_size, width, height, visualize=False):
#         """ Apply camera calibration operation for images in the given directory path. """
#         criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#         objp = np.zeros((height * width, 3), np.float32)
#         objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
#         objp = objp * square_size
#         objpoints = []
#         imgpoints = []

#         images = os.listdir(dirpath)

#         for fname in images:
#             img = cv2.imread(os.path.join(dirpath, fname))
#             gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#             ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

#             if ret:
#                 objpoints.append(objp)
#                 corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#                 imgpoints.append(corners2)

#                 if visualize:
#                     img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
#                     cv2.imshow('img', img)
#                     cv2.waitKey(0)

#         ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#         return [ret, mtx, dist, rvecs, tvecs]

#     def image_callback(self, data):
#         # 압축 해제하여 이미지 데이터를 numpy 배열로 변환
#         np_arr = np.frombuffer(data.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # 카메라 캘리브레이션 적용
#         image_np = cv2.undistort(image_np, self.calibration_matrix, self.distortion_coefficients)

#         # ArUco 마커 감지
#         gray = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
#         aruco_params = cv2.aruco.DetectorParameters_create()
#         corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=aruco_params)

#         if ids is not None and len(ids) > 0:
#             for i in range(len(ids)):
#                 # ArUco 마커의 포즈(위치 및 방향) 추정
#                 rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.08, self.calibration_matrix, self.distortion_coefficients)

#                 # 표시할 문자열 생성
#                 pose_info = f"ID: {ids[i]}, X: {tvec[0][0]:.2f}, Y: {tvec[0][1]:.2f}, Z: {tvec[0][2]:.2f}"

#                 # 이미지에 표시
#                 cv2.aruco.drawAxis(image_np, self.calibration_matrix, self.distortion_coefficients, rvec, tvec, 0.1)
#                 cv2.putText(image_np, pose_info, (10, 30 * (i + 1)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # OpenCV를 사용하여 이미지를 표시
#         cv2.imshow("Compressed Image with ArUco", image_np)
#         cv2.waitKey(1)  # OpenCV 창이 바로 닫히지 않도록 1밀리초 대기

# def main(args=None):
#     rclpy.init(args=args)
#     image_subscriber = CompressedImageSubscriber()
#     rclpy.spin(image_subscriber)
#     # 프로그램 종료 시 OpenCV 창을 정리
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()