# import os
import sys
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time, datetime
from serial import Serial
import mysql.connector

# DB 엑세스 정보 가져오는 함수
# 엑세스 정보는 담당 팀원에게 요청
def import_aws_rds_access_key(file_path):
    access_key_info = []

    try:
        with open(file_path, 'r') as file:
            lines = file.read().split('\n')

            for line in lines:
                if line.strip():
                    access_key_info.append(line)
    
    except Exception as e:
        print(f"파일 읽기 오류: {str(e)}")
    
    return access_key_info

# UI 파일 불러오기
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/gui_4_controlPC.ui")[0]

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

# 로봇의 요청 및 상태
bots_request = ["HUB", "B1", "B2", "B3"]
bots_status = ["사용가능", "사용불가", "이동 중", "사용 중", "충전 중", "신호없음"]

class CharingBot:
    def __init__(self, id, request = "HUB", status = "신호없음", battery_remain = 0):
        self.id = id
        self.request = request
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

        self.arduino_timer = QTimer(self)
        self.arduino_timer.timeout.connect(self.parking_lot_status)  # start timeout 시에 연결할 함수. 즉, 1초마다 상태값을 가져오는 함수를 호출.
        self.arduino_timer.start(100)

        # 로봇들의 상태를 관리 및 지정하는 파트
        self.M1 = CharingBot(1, bots_request[3], bots_status[3], 50)
        self.M2 = CharingBot(2, bots_request[1], bots_status[2], 77)
        self.M3 = CharingBot(3, bots_request[0], bots_status[-2], 7)
        self.Ms = [self.M1, self.M2, self.M3]
        self.M_labels = [self.m1_status, self.m2_status, self.m3_status]
        self.M_tables = [self.m1_table, self.m2_table, self.m3_table]

        self.robot_timer = QTimer(self)
        self.robot_timer.timeout.connect(self.robot_status)
        self.robot_timer.start(100)

        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        # 카메라 선택 구역 라디오 버튼
        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.toggled.connect(self.display_radio_clicked)
        self.minibot2_convert.toggled.connect(self.display_radio_clicked)
        self.minibot3_convert.toggled.connect(self.display_radio_clicked)

        # 테이블 위젯 스케일링
        self.database.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m1_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m1_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m2_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m2_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m3_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m3_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.DB_timer = QTimer(self)
        self.DB_timer.timeout.connect(self.update_parking_system_log)
        self.DB_timer.start(5000)
    
    def update_parking_system_log(self):
        aws_rds_access_key = import_aws_rds_access_key("/home/wintercamo/Documents/aws_rds_access_key.txt")

        remote = mysql.connector.connect(
            host = aws_rds_access_key[0],
            port = int(aws_rds_access_key[1]),
            user = aws_rds_access_key[2],
            password = aws_rds_access_key[3],
            database = aws_rds_access_key[4]
        )
        
        remote_cur = remote.cursor()
        remote_cur.execute("SELECT * FROM park_system_log")

        result = remote_cur.fetchall()

        # 현재 데이터베이스의 행 수와 열 수 가져오기
        current_rows = self.database.rowCount()
        current_cols = self.database.columnCount()

        # 데이터베이스에서 가져온 결과와 현재 테이블 위젯의 데이터 비교 및 업데이트
        for row_num, row_data in enumerate(result):
            if row_num < current_rows:
                # 현재 행이 있으면 업데이트
                for col_num, col_data in enumerate(row_data):
                    if col_num < current_cols:
                        item = self.database.item(row_num, col_num)
                        if item and item.text() != str(col_data):
                            item.setText(str(col_data))
                    else:
                        # 열 수가 현재 테이블 위젯보다 많을 경우 새로운 열 추가
                        if isinstance(col_data, datetime.datetime):
                            col_data = col_data.strftime("%d-%H:%M")
                            print(str(col_data))
                        self.database.setItem(row_num, col_num, QTableWidgetItem(str(col_data)))
                continue

            # 현재 행이 없으면 새로운 행 추가
            row = self.database.rowCount()
            self.database.insertRow(row)
            for col_num, col_data in enumerate(row_data):
                if isinstance(col_data, datetime.datetime):
                    col_data = col_data.strftime("%d-%H:%M")
                    print(str(col_data))
                self.database.setItem(row, col_num, QTableWidgetItem(str(col_data)))

        remote.close()

    # def robot_schedule(self, text):
    #     print(text)
    #     # bots_status = ["0 사용가능", "1 사용불가", "2 이동 중", "3 사용 중", "4 충전 중", "5 신호없음"]

    #     for M, table in zip(self.Ms, self.M_tables):
            
    #         # if M.battery_remain == 0:  # 배터리 없으면 사용불가 상태
    #         #     M.status = bots_status[1]
        
    #         if M.status == bots_status[0]:  # HUB에서 대기 중일 경우 
    #             request = "대기 중"; destination = "-"

    #         elif M.status == bots_status[1]:  # 배터리가 없어서 충전이 필요할 경우.
    #             request = "사용불가"; destination = "HUB"
                

    #         elif M.status == bots_status[2]:  # 이동 중
    #             destination = text; request = f"{destination}으로 이동 중"

    #         elif M.status == bots_status[3]:  # 전기차를 충전 중인 경우
    #             destination = text; request = f"{destination} 차량이 사용 중"
                
    #             if M.battery_remain > -1:
    #                 M.battery_remain -= 1
    #             else:
    #                 M.status = bots_status[1]

    #         elif M.status == bots_status[4]:  # 로봇 스스로가 충전 중인 경우
    #             destination = "-"; request = "충전 중"

    #             if M.battery_remain != 100:
    #                 M.battery_remain += 1
    #             else:
    #                 M.status = bots_status[0]  # 충전 끝나면 사용가능으로 변경.
    
    def robot_status(self):
        for M, label, table in zip(self.Ms, self.M_labels, self.M_tables):
            # bots_request = ["HUB", "B1", "B2", "B3"]
            # bots_status = ["사용가능", "사용불가", "이동 중", "사용 중", "충전 중", "신호없음"]

            if M.status == bots_status[0]:  # Hub에서 대기 중
                status = "사용가능"; color = "#05fa2e"
                

            elif M.status == bots_status[1] or M.status == bots_status[-1] or M.status == bots_status[4]:
                status = "사용불가"; color = "#fa0505"

            elif M.status == bots_status[2] or M.status == bots_status[3]:
                status = "사용 중"; color = "#f6fa05"

            label.setText(status)
            label.setStyleSheet(f"color: black; background-color: {color};")
        
            # table.setRowCount(3)
            # table.setColumnCount(1)
            table.setItem(0,0, QTableWidgetItem(str(M.request)))
            table.setItem(1,0, QTableWidgetItem(str(M.status)))
            table.setItem(2,0, QTableWidgetItem(str(M.battery_remain)))

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

        empty_lots = int(serial_buffer[0]) + int(serial_buffer[1]) + int(serial_buffer[2])

        self.park_num.setText(f"{empty_lots} 대"); self.park_num.setStyleSheet("color: #f6d32d; background-color: transparent;")

    def display_radio_clicked(self):
        if self.cctv_convert.isChecked():
            self.CCTVstart()
        
        else:
            self.CCTVstop()

        if self.minibot1_convert.isChecked():
            self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: black; background-color: lightgreen;")

        else:
            self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: #f6d32d;")
        
        if self.minibot2_convert.isChecked():
            self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: black; background-color: lightgreen;")

        else:
            self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: #f6d32d;")
          
        if self.minibot3_convert.isChecked():
            self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: black; background-color: lightgreen;")

        else:
            self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: #f6d32d;")
     
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
    app = QApplication(sys.argv)
    window = windowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()