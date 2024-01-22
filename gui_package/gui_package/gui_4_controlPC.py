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
def create_DB_cursor(file_path):
    access_key_info = []

    try:
        with open(file_path, 'r') as file:
            lines = file.read().split('\n')

            for line in lines:
                if line.strip():
                    access_key_info.append(line)
    
    except Exception as e:
        print(f"파일 읽기 오류: {str(e)}")

    remote = mysql.connector.connect(
       host = access_key_info[0],
        port = int(access_key_info[1]),
        user = access_key_info[2],
        password = access_key_info[3],
        database = access_key_info[4]
    )

    return remote

access_key = "/home/wintercamo/Documents/aws_rds_access_key.txt"

# 로봇의 요청 및 상태
bots_status = {"사용가능":["대기 중"],
               "사용불가":["충전필요", "수리 중"],
               "사용 중":["이동 중", "사용 중"]}
bots_request = ["HUB", "B1", "B2", "B3"]

class CharingBot:
    def __init__(self, id, status = "사용가능", status_detail = bots_status["사용가능"][0], request=bots_request[0], battery_remain = 0):
        self.id = id
        self.status = status
        self.status_detail = status_detail
        self.request = request
        self.isArrived = True  # True: 목적지에 도착, False: 이동 중
        self.battery_remain = battery_remain
    
    def __str__(self):
        return f'''{self.id}번 로봇\n
                    상태:{self.status} {self.status_detail}\n
                    요청:{self.request}\n
                    도착:{self.request}\n
                    잔여량: {self.battery_remain}
                '''

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

# 고객의 주차 여부를 파악하기 위한 Arduino 시리얼을 받아오는 스레드 클래스
class ArduinoSerial(QThread):
    receive = pyqtSignal(str)

    def __init__(self, serial=None):
        super().__init__()
        self.serial = serial
        self.running = True

    def run(self):
        # global serial_buffer
        while self.running == True:
            try:
                if self.serial.readable():
                    serial_buffer = self.serial.readline().decode('utf-8').strip()
                    self.receive.emit(serial_buffer)
            except:
                print('Waiting...')

# GUI 클래스
class ControlPCWindow(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Voltie Manager')
        
        # 주자창에 주차된 차를 파악하기 위한 MCU 설정
        self.arduino_conn = Serial(port='/dev/ttyACM0', baudrate=9600)
        self.arduino = ArduinoSerial(self.arduino_conn)
        self.arduino.receive.connect(self.parking_lot_status)
        self.arduino.start()
        self.isOuppied = []

        # 로봇들의 상태를 관리 및 지정하는 파트
        self.M_status = [self.m1_status, self.m2_status, self.m3_status]
        self.M_tables = [self.m1_table, self.m2_table, self.m3_table]

        self.bot_timer = QTimer(self)
        self.bot_timer.timeout.connect(self.bots_status)
        self.bot_timer.start(100)

        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.cctv_pixmap = QPixmap()
        self.cctv = CCTVCam()
        self.cctv.update.connect(self.updateCCTV)

        # 카메라 선택 구역 라디오 버튼
        self.cctv_convert.clicked.connect(self.display_radio_clicked)
        self.minibot1_convert.toggled.connect(self.display_radio_clicked)
        self.minibot2_convert.toggled.connect(self.display_radio_clicked)
        self.minibot3_convert.toggled.connect(self.display_radio_clicked)

    #     # 테이블 위젯 스케일링
        self.database.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m1_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m1_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m2_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m2_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m3_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.m3_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
        self.park_DB_timer = QTimer(self)
        self.park_DB_timer.timeout.connect(self.update_parking_system_log)
        self.park_DB_timer.start(5000)

    def update_parking_system_log(self):
        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()
        
        sql_query = f"SELECT * FROM park_system_log"
        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()

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
                    # else:
                    #     # 열 수가 현재 테이블 위젯보다 많을 경우 새로운 열 추가
                    #     if isinstance(col_data, datetime.datetime):
                    #         col_data = col_data.strftime("%d-%H:%M")
                    #     self.database.setItem(row_num, col_num, QTableWidgetItem(str(col_data)))

            # 현재 행이 없으면 새로운 행 추가
            row = self.database.rowCount()
            self.database.insertRow(row)
            for col_num, col_data in enumerate(row_data):
                if isinstance(col_data, datetime.datetime):
                    col_data = col_data.strftime("%d-%H:%M")
                self.database.setItem(row, col_num, QTableWidgetItem(str(col_data)))

        self.remote.close()

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
    
    # def customEvent(self, event):
    #     if event.type() == ImageEvent.EVENT_TYPE:
    #         self.setImage(event.img)

    # def setImage(self, img):
    #     self.display.setPixmap(QPixmap.fromImage(img))

    def display_radio_clicked(self):
        # 라디오 버튼 체크 항목 반영 전 초기화
        if self.isCCTVon == True:
            self.CCTVstop()

        ########################################
        #                                      #
        # 모든 로봇의 화면을 OFF하는 코드를 작성.     #
        #                                      #
        ########################################

    
        self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: #f6d32d;")
        self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: #f6d32d;")
        self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: #f6d32d;")

        # CCTV 라디오가 체크 될 경우
        if self.cctv_convert.isChecked():
            self.CCTVstart()
        
        # 로봇1이 체크 될 경우
        if self.minibot1_convert.isChecked():
            self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: black; background-color: lightgreen;")

            ########################################
            #                                      #
            # 여기에 1번 로봇의 화면을 보여주는 코드를 작성.#
            #                                      #
            ########################################

        # 로봇2가 체크 될 경우
        if self.minibot2_convert.isChecked():
            self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: black; background-color: lightgreen;")

            ########################################
            #                                      #
            # 여기에 2번 로봇의 화면을 보여주는 코드를 작성.#
            #                                      #
            ########################################

        # 로봇3가 체크 될 경우  
        if self.minibot3_convert.isChecked():
            self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: black; background-color: lightgreen;")

            ########################################
            #                                      #
            # 여기에 1번 로봇의 화면을 보여주는 코드를 작성.#
            #                                      #
            ########################################
    
    def CCTVstart(self):
        self.cctv.cctv_running = True
        self.cctv.start()
        # self.video = cv2.VideoCapture('/dev/YourWebCam')
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

            self.cctv_pixmap = self.cctv_pixmap.fromImage(qimage)
            self.cctv_pixmap = self.cctv_pixmap.scaled(self.display.width(), self.display.height())
            
            self.display.setPixmap(self.cctv_pixmap)

    def bots_status(self):
        # DB 연결
        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()
        sql_query = f"SELECT * FROM robot_status"
        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()
        
        for info, table, status in zip(result, self.M_tables, self.M_status):
            
            if info[1] == "사용가능":
                color = "#05fa2e"  # 초록
            elif info[1] == "사용 중":
                color = "#f6fa05"  # 노랑
            else: # 사용불가
                color = "#fa0505"  # 빨강

            status.setText(info[1])
            status.setStyleSheet(f"color: black; background-color: {color};")

            table.setItem(0,0, QTableWidgetItem(info[2]))
            table.setItem(1,0, QTableWidgetItem(info[3]))
            table.setItem(2,0, QTableWidgetItem(str(info[4])))

        self.remote.close()

    def parking_lot_status(self, data):
        if len(data) == 4:  # 간혹 시리얼을 일부만 가져오는 경우가 있기 때문에...
            self.isOuppied = [int(data[0]), int(data[1]), int(data[2]), int(data[3])]

            self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: lightblue;")
            self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: lightblue;")
            self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: lightblue;")
            # print(self.isOuppied)
            if self.isOuppied[0] == 0:
                self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: #f0c5c6;")

            if self.isOuppied[1] == 0:
                self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: #f0c5c6;")

            if self.isOuppied[2] == 0:
                self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: #f0c5c6;")

            self.park_num.setText(f"{self.isOuppied[3]} 대"); self.park_num.setStyleSheet("color: #f6d32d; background-color: transparent;")

def main():
    app = QApplication(sys.argv)
    control_PC_window = ControlPCWindow()
    control_PC_window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()