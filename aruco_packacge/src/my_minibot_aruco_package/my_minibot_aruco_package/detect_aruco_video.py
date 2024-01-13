# '''
# Sample Command:-
# python detect_aruco_video.py --type DICT_5X5_100 --camera True
# python detect_aruco_video.py --type DICT_5X5_100 --camera False --video test_video.mp4
# '''

# import numpy as np
# from aruco_processing import ARUCO_DICT, aruco_display
# import argparse
# import time
# import cv2
# import sys


# ap = argparse.ArgumentParser()
# ap.add_argument("-i", "--camera", required=True, help="Set to True if using webcam")
# ap.add_argument("-v", "--video", help="Path to the video file")
# ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
# args = vars(ap.parse_args())

# if args["camera"].lower() == "true":
# 	video = cv2.VideoCapture('/dev/video0')
# 	time.sleep(2.0)
	
# else:
# 	if args["video"] is None:
# 		print("[Error] Video file location is not provided")
# 		sys.exit(1)

# 	video = cv2.VideoCapture(args["video"])

# if ARUCO_DICT.get(args["type"], None) is None:
# 	print(f"ArUCo tag type '{args['type']}' is not supported")
# 	sys.exit(0)

# arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
# arucoParams = cv2.aruco.DetectorParameters_create()

# while True:
# 	ret, frame = video.read()
	
# 	if ret is False:
# 		break


# 	h, w, _ = frame.shape

# 	width=1000
# 	height = int(width*(h/w))
# 	frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_CUBIC)
# 	corners, ids, rejected = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

# 	detected_markers = aruco_display(corners, ids, rejected, frame)

# 	cv2.imshow("Image", detected_markers)

# 	key = cv2.waitKey(1) & 0xFF
# 	if key == ord("q"):
# 	    break

# cv2.destroyAllWindows()
# video.release()

import numpy as np
import cv2
import sys
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from aruco_processing import ARUCO_DICT, aruco_display

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if len(corners) > 0:
        for i in range(len(ids)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.022, matrix_coefficients, distortion_coefficients)
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

    def image_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        output = pose_estimation(image_np, self.aruco_dict_type, self.k, self.d)
        cv2.imshow('Estimated Pose', output)
        cv2.waitKey(1)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-k", "--K_Matrix", required=True, help="Path to calibration matrix file")
    ap.add_argument("-d", "--D_Coeff", required=True, help="Path to distortion coefficients file")
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
    args = vars(ap.parse_args())

    if ARUCO_DICT.get(args["type"], None) is None:
        print(f"ArUCo tag type '{args['type']}' is not supported")
        sys.exit(0)
    
    k = np.load(args["K_Matrix"])
    d = np.load(args["D_Coeff"])
    aruco_dict_type = ARUCO_DICT[args["type"]]

    rclpy.init(args=None)
    # v4l2 카메라 초기화
    video = cv2.VideoCapture('/dev/video0') # v4l2 카메라 경로 설정
    node = PoseEstimationNode(aruco_dict_type, k, d)
    rclpy.spin(node)
    node.destroy_node()
    video.release() # 카메라 자원 해제
    rclpy.shutdown()

if __name__ == '__main__':
    main()