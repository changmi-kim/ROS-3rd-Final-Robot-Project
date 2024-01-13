# import numpy as np
# import cv2
# import os
# import argparse
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage


# def calibrate(dirpath, square_size, width, height, visualize=False):
#     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
#     objp = np.zeros((height * width, 3), np.float32)
#     objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
#     objp = objp * square_size

#     objpoints = []
#     imgpoints = []

#     images = os.listdir(dirpath)

#     for fname in images:
#         img = cv2.imread(os.path.join(dirpath, fname))
#         gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

#         ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

#         if ret:
#             objpoints.append(objp)

#             corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#             imgpoints.append(corners2)

#             img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
#         if visualize:
                
#             cv2.imshow('img', img)
#             cv2.waitKey(0)

#     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

#     return [ret, mtx, dist, rvecs, tvecs]


# class CompressedImageSubscriber(Node):
#     def __init__(self, calibration_data):
#         super().__init__('compressed_image_subscriber')
#         self.calibration_data = calibration_data
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'image_raw/compressed',
#             self.image_callback,
#             10)
#         self.subscription  # prevent unused variable warning
        
#     def image_callback(self, data):
#         # 압축 해제하여 이미지 데이터를 numpy 배열로 변환
#         np_arr = np.frombuffer(data.data, np.uint8)
#         image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

#         # TODO: 캘리브레이션 데이터를 사용하여 이미지 처리 수행
#         # 예: image_np = cv2.undistort(image_np, self.calibration_data[1], self.calibration_data[2])

#         # OpenCV를 사용하여 이미지를 표시
#         cv2.imshow("Compressed Image", image_np)
#         cv2.waitKey(1)
#     # def image_callback(self, msg):
#     #     # 여기에 이미지 처리 코드를 추가하세요
#     #     # 사용할 calibration_data를 이용하여 이미지 처리 수행
#     #     pass


# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("-d", "--dir", required=True, help="Path to folder containing checkerboard images for calibration")
#     ap.add_argument("-w", "--width", type=int, help="Width of checkerboard (default=8)", default=8)
#     ap.add_argument("-t", "--height", type=int, help="Height of checkerboard (default=6)", default=6)
#     ap.add_argument("-s", "--square_size", type=float, default=1, help="Length of one edge (in metres)")
#     ap.add_argument("-v", "--visualize", type=str, default="False", help="To visualize each checkerboard image")
#     args = vars(ap.parse_args())

#     dirpath = args['dir']
#     square_size = args['square_size']
#     width = args['width']
#     height = args['height']

#     if args["visualize"].lower() == "true":
#         visualize = True
#     else:
#         visualize = False

#     # 캘리브레이션 수행
#     ret, mtx, dist, rvecs, tvecs = calibrate(dirpath, square_size, visualize=visualize, width=width, height=height)

#     print(f"camera rotation matrix : {mtx}")
#     print(f"dist : {dist}")

#     np.save("calibration_matrix", mtx)
#     np.save("distortion_coefficients", dist)

#     # ROS 2 노드 초기화 및 실행

#     rclpy.init()
#     image_subscriber = CompressedImageSubscriber(calibration_data=(ret, mtx, dist, rvecs, tvecs))
#     rclpy.spin(image_subscriber)
#     image_subscriber.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

# import numpy as np
# import cv2
# import os
# import argparse
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from aruco_processing import ARUCO_DICT, aruco_display
# import sys


# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("-i", "--image", required=True, help="path to input image containing ArUCo tag")
#     ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
#     ap.add_argument("-o", "--output", required=True, help="path to output folder to save ArUCo tag")
#     args = vars(ap.parse_args())

#     # ArUCo 마커 이미지 읽기
#     print("Loading image...")
#     cur_dir = os.getcwd()
#     image = cv2.imread(os.path.join(cur_dir, args["image"]))

#     # 이미지 전처리
#     h, w, _ = image.shape
#     width = 600
#     height = int(width * (h / w))
#     image = cv2.resize(image, (width, height), interpolation=cv2.INTER_CUBIC)

#     # verify that the supplied ArUCo tag exists and is supported by OpenCV
#     if ARUCO_DICT.get(args["type"], None) is None:
#         print(f"ArUCo tag type '{args['type']}' is not supported")
#         sys.exit(0)

#     # load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
#     print("Detecting '{}' tags....".format(args["type"]))
#     arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
#     arucoParams = cv2.aruco.DetectorParameters_create()
#     corners, ids, rejected = cv2.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)

#     print(f"corners : {corners}")
#     print(f"ids : {ids}")

#     # ArUCo 마커 검출 결과에 대한 시각화
#     detected_markers = aruco_display(corners, ids, rejected, image)

#     # 저장할 폴더 경로 생성
#     save_dir = os.path.join(cur_dir, args["output"])

#     # 저장할 폴더가 없다면, 해당 폴더 생성
#     try:
#         os.makedirs(save_dir)
#     except OSError:
#         if not os.path.isdir(save_dir):
#             raise

#     # aruco 마커 이미지 저장
#     img_name_split = args["image"].split('/')
#     tag_name = f'{img_name_split[-1]}_detect_result.png'
#     cv2.imwrite(os.path.join(save_dir, tag_name), detected_markers)

#     # 캘리브레이션 데이터 로드
#     # TODO: 캘리브레이션 데이터는 어디에서 가져올지에 따라 적절한 코드로 대체해야 합니다.
#     ret = np.load("calibration_matrix.npy")
#     mtx = ret[1]
#     dist = np.load("distortion_coefficients.npy")

#     # 이미지 보정
#     undistorted_image = cv2.undistort(image, mtx, dist, None, None)

#     # 보정된 이미지 표시
#     cv2.imshow("Detected ArUCo Markers", detected_markers)
#     cv2.imshow("Undistorted Image", undistorted_image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()

import numpy as np
import cv2
import os
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from aruco_processing import ARUCO_DICT, aruco_display


def calibrate(dirpath, square_size, width, height, visualize=False):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((height * width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)
    objp = objp * square_size

    objpoints = []
    imgpoints = []

    images = os.listdir(dirpath)

    for fname in images:
        img = cv2.imread(os.path.join(dirpath, fname))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

        if ret:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
        if visualize:
            cv2.imshow('img', img)
            cv2.waitKey(0)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    return [ret, mtx, dist, rvecs, tvecs]


class CompressedImageSubscriber(Node):
    def __init__(self, calibration_data):
        super().__init__('compressed_image_subscriber')
        self.calibration_data = calibration_data
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

    def image_callback(self, data):
        # 압축 해제하여 이미지 데이터를 numpy 배열로 변환
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # 캘리브레이션 데이터를 사용하여 이미지 보정
        undistorted_image = cv2.undistort(image_np, self.calibration_data[1], self.calibration_data[2])

        # ARUCO 마커 검출
        arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT["DICT_ARUCO_ORIGINAL"])
        arucoParams = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(undistorted_image, arucoDict, parameters=arucoParams)

        # ArUCo 마커 검출 결과에 대한 시각화
        detected_markers = aruco_display(corners, ids, rejected, undistorted_image)

        # OpenCV를 사용하여 이미지 표시
        cv2.imshow("Compressed Image", image_np)
        cv2.imshow("Undistorted Image", undistorted_image)
        cv2.imshow("Detected ArUCo Markers", detected_markers)
        cv2.waitKey(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-d", "--dir", required=True, help="Path to folder containing checkerboard images for calibration")
    ap.add_argument("-w", "--width", type=int, help="Width of checkerboard (default=8)", default=8)
    ap.add_argument("-t", "--height", type=int, help="Height of checkerboard (default=6)", default=6)
    ap.add_argument("-s", "--square_size", type=float, default=1, help="Length of one edge (in metres)")
    ap.add_argument("-v", "--visualize", type=str, default="False", help="To visualize each checkerboard image")
    args = vars(ap.parse_args())

    dirpath = args['dir']
    square_size = args['square_size']
    width = args['width']
    height = args['height']

    if args["visualize"].lower() == "true":
        visualize = True
    else:
        visualize = False

    # 캘리브레이션 수행
    ret, mtx, dist, rvecs, tvecs = calibrate(dirpath, square_size, visualize=visualize, width=width, height=height)

    print(f"camera rotation matrix : {mtx}")
    print(f"dist : {dist}")

    np.save("calibration_matrix", mtx)
    np.save("distortion_coefficients", dist)

    # ROS 2 노드 초기화 및 실행
    rclpy.init()
    image_subscriber = CompressedImageSubscriber(calibration_data=(ret, mtx, dist, rvecs, tvecs))
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


#(aruco) kang@kang-pc:~/minibot_aruco/src/my_minibot_aruco_package/my_minibot_aruco_package$ python pose_estimation.py -k calibration_matrix.npy -d distortion_coefficients.npy
