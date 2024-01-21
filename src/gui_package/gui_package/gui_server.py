import os
import sys
import cv2
import numpy as np
import time, datetime
import socket
from threading import Thread
from collections import deque

import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *



# class CameraThread(QThread):

#     def __init__(self, camera_info):
#         super().__init__()

#         self.deque_size = 1
#         self.deque = deque(maxlen=self.deque_size)

#         self.buffer_size = 16

#         self.minibot_camera = camera_info

#         self.display = False
#         self.cam0 = False
#         self.cam1 = False
#         self.cam2 = False
#         self.cam3 = False


#         self.load_camera(camera_info)



#     def load_camera(self, camera_info):
#         pass
                



# UI 파일 불러오기
from_class = uic.loadUiType("src/gui_package/gui/gui_node.ui")[0]


class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Voltie')

        self.minibot1_convert.hide()
        self.minibot2_convert.hide()
        self.minibot3_convert.hide()

        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.clicked.connect(self.display_radio_clicked)
        self.minibot2_convert.clicked.connect(self.display_radio_clicked)
        self.minibot3_convert.clicked.connect(self.display_radio_clicked)

    
    def button_hide_and_show(self, num):
        if num == 1:
            self.minibot1_convert.show()
            self.minibot2_convert.hide()
            self.minibot3_convert.hide()

        elif num == 2:
            self.minibot2_convert.show()
            self.minibot3_convert.hide()


        elif num == 3:
            self.minibot3_convert.show()

        elif num == 0:
            self.minibot1_convert.hide()
            self.minibot2_convert.hide()
            self.minibot3_convert.hide()
            

    
    def display_radio_clicked(self):
        if self.cctv_convert.isChecked():
            if self.isBot1on == True:

                self.bot1_vision.stop()
                self.bot1_vision.quit()
                self.bot1_vision.wait()
                self.isBot1on = False

            if self.isCCTVon == False:
                self.isCCTVon = True
                self.CCTVstart()

    
    def CCTVstart(self):
        self.cctv.cctv_running = True
        self.cctv.start()
        self.video = cv2.VideoCapture(0)


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



class ImageEvent(QEvent):
    EVENT_TYPE = QEvent.Type(QEvent.registerEventType())

    def __init__(self, img):
        super().__init__(ImageEvent.EVENT_TYPE)
        self.img = img


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



class OpenServer():

    def __init__(self):
        self.HOST = "192.168.1.7" # 로컬 IP 주소
        self.PORT = 3306
        self.client_sockets = []


    def run_server(self):
        print(f'Server Start with IP: {self.HOST}')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = (self.HOST, self.PORT) 
        self.server_socket.bind(server_address)
        self.server_socket.listen()

        print("서버 생성 완료")

        try:
            while True:
                client_socket, client_address = self.server_socket.accept()
                client_id = str(client_address[0])
                print(f'연결 수락됨: {self.ip_name[client_address[0]]}')
                print("client_id : ", client_id)

                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

                self.client_sockets.append(client_socket)
                print(f'참여한 클라이언트 수: {len(self.client_sockets)}')

                gui = WindowClass(self.client_sockets)
                gui.button_hide_and_show(len(self.client_sockets))

                
        except KeyboardInterrupt:
            print('서버를 종료합니다.')

            for client_socket in self.client_sockets:
                client_socket.close()

            self.server_socket.close()
            
        
        finally:
            # 어떠한 경우에도 마지막에 서버 소켓을 닫음
            if self.server_socket:
                self.server_socket.close()




def main():
    server = OpenServer()
    

    # 서버를 쓰레드로 시작
    server_thread = Thread(target=server.run_server, args=())
    server_thread.start()

    # 가져올 카메라 정보
    minibot_1 = "192.168.1.7"
    minibot_2 = "192.168.1.14"
    # minibot_3 = 

    app = QApplication(sys.argv)

    window = WindowClass()
    window.show()

    # 서브쓰레드 종료까지 메인쓰레드 대기
    server_thread.join()

    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()