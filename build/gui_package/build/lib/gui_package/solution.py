import sys
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        cv2.cvtColor(image, cv2.COLOR_BGR2RGB, image)
        height, width, channel = image.shape
        bytesPerLine = 3 * width
        qImg = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888)
        QCoreApplication.postEvent(window, ImageEvent(qImg))


class ImageEvent(QEvent):
    EVENT_TYPE = QEvent.Type(QEvent.registerEventType())

    def __init__(self, image):
        super().__init__(ImageEvent.EVENT_TYPE)
        self.image = image


class CameraThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def run(self):
        rclpy.spin(self.node)


class App(QWidget):
    def __init__(self):
        super().__init__()
        self.imageLabel = QLabel(self)
        self.initUI()
        self.node = ImageSubscriber()
        self.cam_thread = CameraThread(self.node)
        self.cam_thread.start()

    def initUI(self):
        vbox = QVBoxLayout(self)
        vbox.addWidget(self.imageLabel)
        self.setLayout(vbox)
        self.setWindowTitle('Camera View')

    def customEvent(self, event):
        if event.type() == ImageEvent.EVENT_TYPE:
            self.setImage(event.image)

    def setImage(self, image):
        self.imageLabel.setPixmap(QPixmap.fromImage(image))

def main():
    rclpy.init(args=None)
    app = QApplication(sys.argv)
    global window 
    window = App()
    window.show()
    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()