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


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from aruco_processing import aruco_processing  # Import the aruco_processing function

class ImgPublisher(Node):
    def __init__(self):
        super().__init__('img_publisher')
        self.publisher = self.create_publisher(Image, '/v4l2_camera', 10)

        time_period = 0.01
        self.timer = self.create_timer(time_period, self.time_callback)
        self.cap = cv2.VideoCapture('/dev/video0')
        self.cv_bridge = CvBridge()

    def time_callback(self):
        ret, frame = self.cap.read()

        # Process the image using aruco_processing function
        processed_image = aruco_processing(frame)

        img = self.cv_bridge.cv2_to_imgmsg(processed_image, "bgr8")
        self.publisher.publish(img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ImgPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




















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

