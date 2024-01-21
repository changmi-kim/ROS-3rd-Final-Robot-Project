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



# UI 파일 불러오기
from_class = uic.loadUiType("src/gui_package/gui/gui_node.ui")[0]


class windowClass(QMainWindow, from_class):
    def __init__(self, camera_link):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Voltie')
        
        self.client_sockets = []

        self.deque = deque(maxlen=1)
        self.camera_stream_link = camera_link

        #카메라 정보 가져오기
        self.load_camera()

        # 프레임 읽어오기
        self.get_frame_thread = Thread(target=self.get_frame, args=())
        self.get_frame_thread.daemon = True
        self.get_frame_thread.start()

        self.v1 = False
        self.v2 = False
        self.v3 = False


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

        self.isBot1on = False



    # 영상을 받아서 쓰레드로 시작
    def load_camera(self):
        self.load_stream_thread = Thread(target=self.image_callback_thread, args=())
        self.load_stream_thread.daemon = True
        self.load_stream_thread.start()


    def image_callback_thread(self):


    def get_frame(self):
        """Reads frame, resizes, and converts image to pixmap"""

        while True:
            try:
                if self.v1:
                    # Read next frame from stream and insert into deque
                    status, frame = self.capture.read()
                    if status:
                        self.deque.append(frame)
                    else:
                        self.capture.release()
                        self.online = False
                else:
                    # Attempt to reconnect
                    print('attempting to reconnect', self.camera_stream_link)
                    self.load_network_stream()
                    self.spin(2)
                self.spin(.001)
            except AttributeError:
                pass



class OpenServer():

    def __init__(self):
        self.HOST = "192.168.1.7" # 로컬 IP 주소
        self.PORT = 3306


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

                print(f'참여한 클라이언트 수: {len(self.client_sockets)}')
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

                with self.client_sockets_lock:
                    self.client_sockets.append(client_socket)

                # 쓰레드 종료 플래그 생성 또는 재설정
                if client_id in self.thread_stop_flags:
                    self.thread_stop_flags[client_id].set()  # 기존 쓰레드 종료 신호

                    if client_id in self.threads:
                        self.threads[client_id].join()  # 종료될 때까지 기다리기

                self.thread_stop_flags[client_id] = threading.Event()

                # 클라이언트별 쓰레드 시작
                self.threads[client_id] = threading.Thread(
                                                            target=self.image_callback, 
                                                            args=(client_socket, client_id, self.thread_stop_flags[client_id])
                                                            )
                self.threads[client_id].start()

                
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
    minibot_1 = 
    minibot_2 = 
    minibot_3 = 

    # 서브쓰레드 종료까지 메인쓰레드 대기
    server_thread.join()


if __name__ == "__main__":
    main()