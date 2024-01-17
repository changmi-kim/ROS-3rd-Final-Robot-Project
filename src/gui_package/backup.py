import os, subprocess
import sys
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time

server_ip = "192.168.1.5"
ros_bashrc_parameter = {11:f"{server_ip}:11811", 12:f"{server_ip}:11812", 13:f"{server_ip}:11813", 14:f"{server_ip}:11814"}  # 임시

def modifyROSEnvironment(cnt):
    os.environ['ROS_DOMAIN_ID'] = str(cnt)
    os.environ['ROS_DISCOVERY_SERVER'] = str(ros_bashrc_parameter[cnt])

    print(f"ROS_DOMAIN_ID set to: {os.environ['ROS_DOMAIN_ID']}")
    print(f"ROS_DISCOVERY_SERVER set to: {os.environ['ROS_DISCOVERY_SERVER']}")

# 예시: 서버 IP, 도메인 ID, 디스커버리 서버 설정: modifyROSEnvironment(server_ip, 11, '192.168.0.59:11811;192.168.0.59:11812')

# UI 파일 불러오기
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/gui_node.ui")[0]

# Raspberry Pi의 Pi Camera(V4l2)를 구독하는 노드
class PiCamSubscriber(Node):
    def __init__(self):
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
        while self.pi_cam_running == True:
            try:
                rp.spin_once(self.node, timeout_sec=0.1)
            except Exception as e:
                print(f"Error in CameraThread: {e}")
    
    def stop(self):
        self.pi_cam_running = False

class CCTVCam(QThread):
    update = pyqtSignal()

    def __init__(self, sec=0, parent=None):
        super().__init__()
        self.main = parent
        self.cctv_running = True

    def run(self):
        while self.cctv_running == True:
            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.cctv_running = False

class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('PiKaChu Manager')
        
        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        # self.botX_vision = [None, None, None]
        # self.isBotXon = [False, False, False]
        self.botX_vision = None
        self.isBotXon = False

        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.clicked.connect(self.display_radio_clicked)
        self.minibot2_convert.clicked.connect(self.display_radio_clicked)
        self.minibot3_convert.clicked.connect(self.display_radio_clicked)
    def display_radio_clicked(self):
        if self.cctv_convert.isChecked():  # CCTV로 화면 전환
            # 로봇들의 화면이 켜있는 상태일 경우, 로봇 화면 종료.
            try:
                self.botX_vision.stop()
                self.botX_vision.quit()
                self.botX_vision.wait()
            except:
                print("켜진 로봇 없음")
            finally:
                self.CCTVstart()

        elif self.minibot1_convert.isChecked():  # 1번 로봇 화면으로 전환
            try:
                self.CCTVstop()
            except:
                print('\n')
            
            try:
                self.botX_vision.stop()
                self.botX_vision.quit()
                self.botX_vision.wait()
            except:
                print("켜진 로봇 없음")
            finally:
                modifyROSEnvironment(11)
                self.node = PiCamSubscriber()
                self.botX_vision = CameraThread(self.node)
                self.botX_vision.start()

        elif self.minibot2_convert.isChecked():  # 2번 로봇 화면으로 전환            
            try:
                self.CCTVstop()
            except:
                print('\n')
            try:
                self.botX_vision.stop()
                self.botX_vision.quit()
                self.botX_vision.wait()                
            finally:
                modifyROSEnvironment(12)
                self.node = PiCamSubscriber()
                self.botX_vision = CameraThread(self.node)
                self.botX_vision.start()        
     
    def CCTVstart(self):
        self.cctv.cctv_running = True
        self.cctv.start()
        self.video = cv2.VideoCapture('/dev/YourWebCam')

    def CCTVstop(self):
        self.cctv.cctv_running = False
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
