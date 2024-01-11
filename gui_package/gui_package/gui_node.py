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
import time, datetime

from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/gui_node.ui")[0]

class PiCamSubscriber(Node):
    def __init__(self):
        super().__init__('pi_cam_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)
        # self.subscription

    def listener_callback(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        cv2.cvtColor(img, cv2.COLOR_BGR2RGB, img)
        h, w, c = img.shape
        bytesPerLine = 3 * w
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

    def run(self):
        rp.spin(self.node)

# class CCTVCam(QThread):
#     update = pyqtSignal()

#     def __init__(self, sec=0, parent=None):
#         super().__init__()
#         self.main = parent
#         self.cctv_running = True

#     def run(self):
#         while self.cctv_running == True:
#             self.update.emit()
#             time.sleep(0.1)
    
#     def stop(self):
#         self.running = False

class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('PiKaChu Manager')

        # self.image = None
        # self.cctv = CCTVCam(self)
        # self.cctv.update.connect(self.updateCCTV)

        self.node = PiCamSubscriber()
        self.cam_thread = CameraThread(self.node)
        
        self.cctv_convert.setChecked(True)
        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.clicked.connect(self.display_radio_clicked)
        self.minibot2_convert.clicked.connect(self.display_radio_clicked)
        self.minibot3_convert.clicked.connect(self.display_radio_clicked)
    
    def display_radio_clicked(self):
        if self.cctv_convert.isChecked():
            # self.CCTVstart()
            pass
        elif self.minibot1_convert.isChecked():
            # self.CCTVstop()
            self.cam_thread.start()
        else:
            pass
        
    # def CCTVstart(self):
    #     if self.cctv.cctv_displayed == False:
    #         self.cctv_displayed = True
    #         self.cctv.start()
    #         self.video = cv2.VideoCapture('/dev/YourWebCam')

    # def CCTVstop(self):
    #     self.cctv.cctv_displayed = False
    #     self.video.release()
        
    # def updateCCTV(self):
    #     ret, self.image = self.video.read()
    #     if ret:
    #         self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
    #         self.now = datetime.datetime.now().strftime('%y/%m/%d _ %H:%M:%S')
    #         cv2.putText(self.image, self.now, (30, 30), cv2.FONT_ITALIC, 1, (255,255,255), 2)

    #         h,w,c = self.image.shape

    #         qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

    #         self.pixmap = self.pixmap.fromImage(qimage)
    #         self.pixmap = self.pixmap.scaled(self.display.width(), self.display.height())
            
    #         self.display.setPixmap(self.display)

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