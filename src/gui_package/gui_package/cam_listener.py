import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from PyQt5 import uic
from PyQt5.QtWidgets import QMainWindow, QApplication
from PyQt5.QtCore import QThread, QEvent, pyqtSignal, QCoreApplication, QMutex, QMutexLocker
from PyQt5.QtGui import QImage, QPixmap

import os
import sys
import cv2
import numpy as np
import time


    
# ROS 통신을 위해 서버 정의
server_ip = "192.168.1.7"
ros_bashrc_parameter = {10:f"{server_ip}:11811;{server_ip}:11812;{server_ip}:11813;{server_ip}:11814", 11:f"{server_ip}:11811", 12:f"{server_ip}:11812", 13:f"{server_ip}:11813", 14:f"{server_ip}:11814"}  # 임시


def modifyROSEnvironment(cnt):
    os.environ['ROS_DOMAIN_ID'] = str(cnt)
    os.environ['ROS_DISCOVERY_SERVER'] = str(ros_bashrc_parameter[cnt])

    print(f"ROS_DOMAIN_ID set to: {os.environ['ROS_DOMAIN_ID']}")
    print(f"ROS_DISCOVERY_SERVER set to: {os.environ['ROS_DISCOVERY_SERVER']}")


# UI 파일 불러오기
from_class = uic.loadUiType("/home/jo/final_commit/src/gui_package/gui/gui_node.ui")[0]



# Raspberry Pi의 Pi Camera(V4l2)를 구독하는 노드
class PiCamSubscriber(Node):
    def __init__(self, node_name):
        super().__init__('pi_cam_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)  # 수신 받은 데이터를 numpy 배열로 변환.
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # image data를 decode
        cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)  # BGR을 RGB로.
        
        # PyQt에 image를 표시하기 위해 QImage 객체 생성.
        h, w, c = img.shape
        bytesPerLine = c * w
        q_img = QImage(img.data, w, h, bytesPerLine, QImage.Format_RGB888)
        QCoreApplication.postEvent(window, ImageEvent(q_img))



class ImageEvent(QEvent):
    EVENT_TYPE = QEvent.Type(QEvent.registerEventType())

    def __init__(self, img):
        super().__init__(ImageEvent.EVENT_TYPE)
        self.img = img



class CameraThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.pi_cam_running = True

        
    def run(self):
        try:
            while self.pi_cam_running:
                rp.spin(self.node)

        except Exception as e:
            print(f"Error in CameraThread: {e}")
    
    def stop(self):
        self.pi_cam_running = False



class CCTVCam(QThread):
    update = pyqtSignal()  # 이름을 update로 변경

    def __init__(self, sec=0, parent=None):
        super().__init__()
        self.main = parent
        self.running = True  # 종료 플래그 추가
        self.mutex = QMutex()

    def run(self):
        try:
            while self.running:  # 종료 플래그 확인
                with QMutexLocker(self.mutex):
                    self.update.emit()  # 신호를 발생시킴 (이름을 update로 변경)

        except Exception as e:
            print(f"Error in CCTVCam: {e}")

    def stop(self):
        self.running = False  # 종료 플래그 설정



class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('PiKaChu Manager')

        
        # CCTV 설정
        self.image = None
        self.pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        self.bot1_vision = None
        self.bot2_vision = None
        self.bot3_vision = None


        self.cctv_convert.toggled.connect(self.start_cctv)
        self.minibot1_convert.toggled.connect(self.display_radio_clicked)
        self.minibot2_convert.toggled.connect(self.display_radio_clicked)
        self.minibot3_convert.toggled.connect(self.display_radio_clicked)


    def display_radio_clicked(self):
        if self.minibot1_convert.isChecked():
            modifyROSEnvironment(12)

            self.node = PiCamSubscriber("minibot1")
            self.bot1_vision = CameraThread(self.node)
            self.bot1_vision.start()

        else:
            if self.bot1_vision is not None:
                self.bot1_vision.stop()
                self.bot1_vision.quit()
                self.bot1_vision.wait()


        if self.minibot2_convert.isChecked():
            modifyROSEnvironment(13)
            self.node = PiCamSubscriber("minibot2")
            self.bot2_vision = CameraThread(self.node)
            self.bot2_vision.start()

        else:
            if self.bot2_vision is not None:
                self.bot2_vision.stop()
                self.bot2_vision.quit()
                self.bot2_vision.wait()


        if self.minibot3_convert.isChecked():
            modifyROSEnvironment(14)
            self.node = PiCamSubscriber("minibot3")
            self.bot3_vision = CameraThread(self.node)
            self.bot3_vision.start()

        else:
            if self.bot3_vision is not None:
                self.bot3_vision.stop()
                self.bot3_vision.quit()
                self.bot3_vision.wait()


    def start_cctv(self):
        #  CCTV Play
        if self.cctv_convert.isChecked():
            self.CCTVstart()
        
        else:
            self.CCTVstop()


    def CCTVstart(self):
        self.cctv.cctv_running = True
        self.cctv.start()
        self.video = cv2.VideoCapture(0)


    def CCTVstop(self):
        self.cctv.cctv_running = False
        self.cctv.stop()
        self.video.release()


    def updateCCTV(self):
        retval, self.image = self.video.read()
        if retval:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

            h,w,c = self.image.shape
            qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.display.width(), self.display.height())
            
            self.display.setPixmap(self.pixmap)


    def customEvent(self, event):
        if event.type() == ImageEvent.EVENT_TYPE:
            self.setImage(event.img)

    def setImage(self, img):
        self.display.setPixmap(QPixmap.fromImage(img))



def main():
    rp.init(args=None)
    app = QApplication(sys.argv)
    global window 
    window = windowClass()
    window.show()
    exit_code = app.exec_()
    rp.shutdown()
    sys.exit(exit_code)



if __name__ == '__main__':
    main()

        
        
