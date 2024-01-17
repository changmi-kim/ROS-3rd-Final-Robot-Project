import os
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
from serial import Serial

server_ip = "192.168.1.5"
ros_bashrc_parameter = {11:f"{server_ip}:11811", 77:f"{server_ip}:11812", 13:f"{server_ip}:11813", 14:f"{server_ip}:11814"}

def modifyROSEnvironment(cnt):
    os.environ['ROS_DOMAIN_ID'] = str(cnt)
    os.environ['ROS_DISCOVERY_SERVER'] = str(ros_bashrc_parameter[cnt])

    print(f"ROS_DOMAIN_ID set to: {os.environ['ROS_DOMAIN_ID']}")
    print(f"ROS_DISCOVERY_SERVER set to: {os.environ['ROS_DISCOVERY_SERVER']}")

# UI 파일 불러오기
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/gui_4_controlPC.ui")[0]

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
        print("되나 테스트 중")
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

# CCTV 용도를 위한 웹캠 스레드 클래스
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

# 로봇 클래스
bots_status = ["사용가능", "사용불가", "이동 중", "사용 중", "충전 중", "신호없음"]

class CharingBot:
    def __init__(self, id, status = "No Signal", battery_remain = 0):
        self.id = id
        self.status = status
        self.battery_remain = battery_remain
    
    def __str__(self):
        return f"{self.id}번 로봇\n{self.status}\n잔여량: {self.battery_remain}"

# GUI 클래스
class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('PiKaChu Manager')
        
        # Arudino 시리얼을 GUI와 연동시키기 위한 설정 (고객 주차장)
        self.arduino_conn = Serial(port='/dev/ttyACM0', baudrate=9600)
        self.arduino = SerialManager(self.arduino_conn)
        self.arduino.start()

        self.parking_lot_widget.setItem(0, 0, QTableWidgetItem("1111"))
        self.parking_lot_widget.setItem(0, 1, QTableWidgetItem("22"))
        self.parking_lot_widget.setItem(1, 2, QTableWidgetItem("33"))

        self.arduino_timer = QTimer(self)
        self.arduino_timer.timeout.connect(self.parking_lot_status)  # start timeout 시에 연결할 함수. 즉, 1초마다 상태값을 가져오는 함수를 호출.
        self.arduino_timer.start(100)

        # 로봇들의 상태를 관리 및 지정하는 파트
        self.M1 = CharingBot(1, bots_status[0], 100)
        self.M2 = CharingBot(2, bots_status[1], 50)
        self.M3 = CharingBot(3, bots_status[2], 12)
        self.Ms = [self.M1, self.M2, self.M3]
        self.M_labels = [self.m1_status, self.m2_status, self.m3_status]

        self.robot_timer = QTimer(self)
        self.robot_timer.timeout.connect(self.robot_status)
        self.robot_timer.start(100)

        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        self.current_thread_cam = None

        # 카메라 선택 구역 라디오 버튼
        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.toggled.connect(self.display_radio_clicked)
        self.minibot2_convert.toggled.connect(self.display_radio_clicked)
        self.minibot3_convert.toggled.connect(self.display_radio_clicked)

        # 테이블 위젯 스케일링
        self.database.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.parking_lot_widget.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
    
    def robot_status(self):
        for M, label in zip(self.Ms, self.M_labels):
            if M.status == bots_status[0]:  # Hub에서 대기 중
                status = "사용가능"
                color = "#05fa2e"
            elif M.status == bots_status[1] or M.status == bots_status[-1]: # 고장난 경우
                status = "사용불가"
                color = "#fa0505"
            elif M.status == bots_status[2] or M.status == bots_status[3] or M.status == bots_status[4]:  # 이동 중
                status = "사용 중"
                color = "#f6fa05"

            label.setText(status)
            label.setStyleSheet(f"color: black; background-color: {color};")

    def parking_lot_status(self):
        global serial_buffer

        self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: lightblue;")
        self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: lightblue;")
        self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: lightblue;")
        
        if int(serial_buffer[0]) == 0:
            self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: #f0c5c6;")

        if int(serial_buffer[1]) == 0:
            self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: #f0c5c6;")

        if int(serial_buffer[2]) == 0:
            self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: #f0c5c6;")

        empty_lots = int(serial_buffer[0]) +  int(serial_buffer[1]) + int(serial_buffer[2])
        
        self.parking_lot_widget.setRowCount(3)

        self.parking_lot_widget.setItem(0, 0, QTableWidgetItem("00"))
        self.parking_lot_widget.setItem(1, 0, QTableWidgetItem("10"))
        self.parking_lot_widget.setItem(0, 1, QTableWidgetItem("01"))
        self.parking_lot_widget.setItem(1, 1, QTableWidgetItem("11"))
        
    def display_radio_clicked(self):
        self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: #f6d32d;")
        self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: #f6d32d;")
        self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: #f6d32d;")
        
        if self.cctv_convert.isChecked():
            
            pass

        elif self.minibot1_convert.isChecked():
            self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: black; background-color: lightgreen;")
        
        elif self.minibot2_convert.isChecked():
            self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: black; background-color: lightgreen;")
          
        elif self.minibot3_convert.isChecked():  
            self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: black; background-color: lightgreen;")
        #         self.CCTVstop()
        #     except:
        #         print('CCTV 안 켜진 상태')

        #     try:
        #         print("다른 로봇 화면 켜있으면 끄기")
        #     except:
        #         print("켜진 로봇 없음")               
        #     finally:
        #         print("2번 로봇 화면 On")
        
        # # 3번 로봇 화면으로 전환
        # elif self.minibot3_convert.isChecked():  
        #     try:
        #         self.CCTVstop()
        #     except:
        #         print('CCTV 안 켜진 상태')

        #     try:
        #         print("다른 로봇 화면 켜있으면 끄기")
        #     except:
        #         print("켜진 로봇 없음")               
        #     finally:
        #         print("3번 로봇 화면 On")
        
        # for i in range(3):
        #     self.the_bots[i].selected = self.minibot1_convert.isChecked()
            
        #     if self.the_bots[i].selected == True:
        #         # QT에 select 되었다는 것을 표시
        #         palette = self.the_bots[i].status_box.palette()
        #         palette.setColor(QPalette.Window, QColor("lightblue"))  # 배경색을 lightblue로 설정
        #         self.the_bots[i].status_box.setPalette(palette)
        #         self.the_bots[i].status_box.setAutoFillBackground(True)

        #     else:
        #         palette = self.the_bots[i].status_box.palette()
        #         palette.setColor(QPalette.Window, QColor("gray"))
        #         self.the_bots[i].status_box.setPalette(palette)
        #         self.the_bots[i].status_box.setAutoFillBackground(True)

        #     self.the_bots[i].selected = ~self.the_bots[i].selected

                

     
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

# 고객의 주차 여부를 파악하기 위한 Arduino 시리얼을 받아오는 스레드 클래스
class SerialManager(QThread):
    receive = pyqtSignal(str)

    def __init__(self, serial=None):
        super().__init__()
        self.serial = serial
        self.running = True

    def run(self):
        global serial_buffer
        while self.running == True:
            try:
                if self.serial.readable():
                    serial_buffer = self.serial.readline().decode('utf-8').strip()
            except:
                print('Waiting...')

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
