import sys
import cv2
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Empty

           
# UI file path
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/visual.ui")[0]
class WindowClass(QMainWindow, from_class):
    def __init__(self, parent=None):
        super().__init__()
        self.setupUi(self)

if __name__ == "__main__":
    app = QApplication(sys.argv)    
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())