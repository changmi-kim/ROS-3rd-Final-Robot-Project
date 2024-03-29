import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import torch
import random
from ctypes import *
import time

from ultralytics import YOLO
from ultralytics.trackers import BOTSORT, BYTETracker
from ultralytics.trackers.basetrack import BaseTrack
from ultralytics.utils import IterableSimpleNamespace, yaml_load
from ultralytics.utils.checks import check_requirements, check_yaml

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_object_detector_msgs.msg import DetectionResult
from vision_object_detector_msgs.msg import DetectionResultArray
from std_srvs.srv import SetBool

LIB_PATH = '/home/ckdal/dev_ws/project/final_project_ws/final_project_ws/src/vision_object_detector/bin/linux-x86_64/libtsanpr.so'
print('LIB_PATH=', LIB_PATH)

lib = cdll.LoadLibrary(LIB_PATH)

lib.anpr_initialize.argtype = c_char_p
lib.anpr_initialize.restype = c_char_p

lib.anpr_read_pixels.argtypes = (c_char_p, c_int32, c_int32, c_int32, c_char_p, c_char_p, c_char_p)
lib.anpr_read_pixels.restype = c_char_p

class ParkingObstacleDetectorYoloV8(Node):
    def __init__(self):
        super().__init__('parking_obstacle_detector_yolo_v8')
        self.subscriber_img = self.create_subscription(Image, '/image_raw', self.yolo_image_callback, qos_profile_sensor_data)
        self.subscriber_control_event = self.create_subscription(String, '/control_event', self.control_event_callback, 10)
        
        # self.publisher_detections = self.create_publisher(DetectionResult, '/detection_result', 10)
        self.publisher_detections = self.create_publisher(DetectionResultArray, '/detection_result', 10)
        self.publisher_debug_image = self.create_publisher(Image, '/debug_image', 10)
        self.publisher_car_number = self.create_publisher(String, '/car_number', 10)

        # 비활성화 시
        # ros2 service call /parking_obstacle_detector_yolo_v8/service_control_enable std_srvs/SetBool "{data: false}"
        self.service_control_enable = self.create_service(SetBool, 'enable', self.enable_callback)

        lib.anpr_initialize(b'text')

        self.declare_parameter('model', 'yolov8m.pt')
        model = self.get_parameter('model').get_parameter_value().string_value

        self.declare_parameter('tracker', 'bytetrack.yaml')
        tracker = self.get_parameter('tracker').get_parameter_value().string_value

        self.declare_parameter('device', 'cuda:0') # "cpu"
        device = self.get_parameter('device').get_parameter_value().string_value

        self.declare_parameter('threshold', 0.75)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.declare_parameter('enable', True)
        self.enable = self.get_parameter('enable').get_parameter_value().bool_value

        self.cv_bridge = CvBridge()
        self.class_to_color = {'car': (0, 0, 255), 'person': (0, 255, 255)}
        self.yolo = YOLO(model)
        self.tracker = self.create_tracker(tracker)
        self.yolo.fuse()
        self.yolo.to(device)

        self.control_start_event = String()

        self.detection = DetectionResult()
        self.detection_msg = DetectionResultArray()

    def control_event_callback(self, msg: String):
        self.control_start_event = msg

    def yolo_image_callback(self, msg: Image) -> None:
        if self.enable:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=0.1,
                mode='track'
            )

            det = results[0].boxes.cpu().numpy()

            if len(det) > 0:
                im0s = self.yolo.predictor.batch[2]
                im0s = im0s if isinstance(im0s, list) else [im0s]

                tracks = self.tracker.update(det, im0s[0])
                if len(tracks) > 0:
                    results[0].update(boxes=torch.as_tensor(tracks[:, :-1]))

            self.detections_msg = DetectionResultArray()

            results = results[0].cpu()

            for b in results.boxes:
                label = self.yolo.names[int(b.cls)]
                score = float(b.conf)

                if score < self.threshold:
                    continue

                if label not in ['car', 'person']:
                    continue

                if self.control_start_event.data == '4' and label == 'car':
                # if label == 'car':
                    car_number_msg = String()
                    
                    height, width, _ = cv_image.shape
                    result = lib.anpr_read_pixels(
                        bytes(cv_image), width, height, 0, b'BGR', b'text', b'')
                    
                    if len(result) > 0:
                        self.get_logger().info(result.decode('utf8'))
                        car_number_msg = String(data = str(result.decode('utf8')))
                        self.publisher_car_number.publish(car_number_msg)

                # print(f'debug: {self.control_start_event}')

                # detection = Detection2D()
                self.detection = DetectionResult()

                box = b.xywh[0]

                self.detection.bbox_center_x = float(box[0])
                self.detection.bbox_center_y = float(box[1])
                self.detection.bbox_size_x = float(box[2])
                self.detection.bbox_size_y = float(box[3])

                track_id = -1
                if not b.id is None:
                    track_id = int(b.id)
                #detection.id = str(track_id)

                self.detection.label = label
                self.detection.score = score
                    
                color = self.class_to_color[label]
                min_pt = (round(self.detection.bbox_center_x - self.detection.bbox_size_x / 2.0),
                          round(self.detection.bbox_center_y - self.detection.bbox_size_y / 2.0))
                max_pt = (round(self.detection.bbox_center_x + self.detection.bbox_size_x / 2.0),
                          round(self.detection.bbox_center_y + self.detection.bbox_size_y / 2.0))
                cv2.rectangle(cv_image, min_pt, max_pt, color, 2)

                label = "{} ({:.3f})".format(label, score)
                pos = (min_pt[0] + 5, min_pt[1] + 25)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(cv_image, label, pos, font, 1, color, 1, cv2.LINE_AA)

                self.detections_msg.detection_result.append(self.detection)

            # self.publisher_detections.publish(self.detection)
            self.publisher_detections.publish(self.detections_msg)
            self.publisher_debug_image.publish(self.cv_bridge.cv2_to_imgmsg(cv_image, encoding = msg.encoding))
        
        cv2.imshow('debug_image', cv_image)
        cv2.waitKey(10)

    def create_tracker(self, tracker_yaml) -> BaseTrack:
        TRACKER_MAP = {'bytetrack': BYTETracker, 'botsort': BOTSORT}
        check_requirements('lap')  # for linear_assignment

        tracker = check_yaml(tracker_yaml)
        cfg = IterableSimpleNamespace(**yaml_load(tracker))

        assert cfg.tracker_type in ['bytetrack', 'botsort'], \
            f"Only support 'bytetrack' and 'botsort' for now, but got '{cfg.tracker_type}'"
        tracker = TRACKER_MAP[cfg.tracker_type](args=cfg, frame_rate=1)
        return tracker

    def enable_callback(self,
                  req: SetBool.Request,
                  res: SetBool.Response
                  ) -> SetBool.Response:
        self.enable = req.data
        res.success = True
        return res

def main(args=None):
    rclpy.init(args=args)
    
    parking_obstacle_detector_yolo_v8 = ParkingObstacleDetectorYoloV8()

    rclpy.spin(parking_obstacle_detector_yolo_v8)

    parking_obstacle_detector_yolo_v8.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()