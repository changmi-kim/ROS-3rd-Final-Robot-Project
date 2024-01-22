import sys
import cv2
import numpy as np
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import datetime, time
from serial import Serial
import mysql.connector

# DB에 접근
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

aws_rds_access_key = import_aws_rds_access_key("/home/wintercamo/Documents/aws_rds_access_key.txt")

remote = mysql.connector.connect(
    host = aws_rds_access_key[0],
    port = int(aws_rds_access_key[1]),
    user = aws_rds_access_key[2],
    password = aws_rds_access_key[3],
    database = aws_rds_access_key[4]
)

remote_cursor = remote.cursor()

# UI 파일 불러오기
from_class1 = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/kiosk.ui")[0]

# GUI 클래스
class KioskWindow(QMainWindow, from_class1):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Kiosk Manager')

        self.groupboxes = [self.phase0, self.phase1, self.phase2, self.phase3, self.phase4, self.phase5]
        self.start_charing = True  # True: 충전시작, False: 충전종료
        self.phase = 0
        self.buffer = ""
        self.captured_img.setText("본인의 차량을\n선택해주세요")
        
        self.update_kiosk_UI()

        # 관제 PC에서 가져올 정보들
        self.isBotsReday = True  # 현재 사용가능한 로봇의 유무

        self.timer = QTimer()
        self.timer.timeout.connect(self.auto_phase_shift)

        # Phase 0
        self.phase0.mousePressEvent = self.phase0_clicked
        # Phase 1
        self.home_btn.clicked.connect(self.home_btn_click)
        self.back_btn.clicked.connect(self.back_btn_click)
        self.charge_start_btn.clicked.connect(self.push_charge_start_btn)
        self.charge_end_btn.clicked.connect(self.push_charge_end_btn)
        # Phase 2
        self.btn9.clicked.connect(self.push_btn9)
        self.btn8.clicked.connect(self.push_btn8)
        self.btn7.clicked.connect(self.push_btn7)
        self.btn6.clicked.connect(self.push_btn6)
        self.btn5.clicked.connect(self.push_btn5)
        self.btn4.clicked.connect(self.push_btn4)
        self.btn3.clicked.connect(self.push_btn3)
        self.btn2.clicked.connect(self.push_btn2)
        self.btn1.clicked.connect(self.push_btn1)
        self.btn0.clicked.connect(self.push_btn0)
        self.btnClear.clicked.connect(self.push_btnClear)
        self.btnConfirm.clicked.connect(self.push_btnConfirm)
        # Phase 3
        self.parked_list.itemSelectionChanged.connect(self.handle_selected_car)
        self.parked_list.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.parked_list.horizontalHeader().setSectionResizeMode(0, QHeaderView.Interactive)
        self.select_car_btn.clicked.connect(self.push_select_car_btn)
        self.pay_cancel_btn.clicked.connect(self.home_btn_click)
        self.pay_btn.clicked.connect(self.pay_confirmed)        

    def push_select_car_btn(self):
        self.captured_img.setText("본인의 차량을\n선택해주세요")
        
        # 충전 시작 상태일 경우
        if self.start_charing:
            self.phase = 4
            self.update_kiosk_UI()

            if self.isBotsReday:
                now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                sql_query = f"UPDATE park_system_log SET charging_start_time = '{now}' WHERE car_number = '{self.target}'"
                remote_cursor.execute(sql_query)
                remote.commit()
                
                self.result_mention.setText(f"충전 로봇이\n{self.target} 차량에\n배정되었습니다.")
            else:
                self.result_mention.setText("죄송합니다.\n현재 로봇들이 충전 중 입니다.")

            self.timer.start(2000)

    # 조회된 차량 목록 중 자신의 차량을 선택
    def handle_selected_car(self):
        selected_items = self.parked_list.selectedItems()
        
        if selected_items:
            selected_row = selected_items[0].row()
            self.target = self.parked_list.item(selected_row, 0).text()
            self.target_time = self.parked_list.item(selected_row, 1).text()
            # 미구현... 입구에서 번호판 촬영하여 S3에 업로드. 그리고 해당 차량에 맞는 이미지를 띄워야하는 부분...
            pixmap = QPixmap("/home/wintercamo/gui_study/src/gui_package/resource/SNOW_20240108_171332_671.jpg")
            self.captured_img.setPixmap(pixmap)
            self.captured_img.setPixmap(pixmap.scaled(self.captured_img.size(), Qt.KeepAspectRatio))
            self.captured_img.setAlignment(Qt.AlignCenter)

    # 차량번호 조회 버튼
    def push_btnConfirm(self):
        if len(self.buffer) == 4:
            self.target_car_search()

    # 차량번호 입력 버튼
    def push_btn9(self):
        if len(self.buffer) < 4: self.buffer += "9"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn8(self):
        if len(self.buffer) < 4: self.buffer += "8"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn7(self):
        if len(self.buffer) < 4: self.buffer += "7"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn6(self):
        if len(self.buffer) < 4: self.buffer += "6"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn5(self):
        if len(self.buffer) < 4: self.buffer += "5"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn4(self):
        if len(self.buffer) < 4: self.buffer += "4"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn3(self):
        if len(self.buffer) < 4: self.buffer += "3"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn2(self):
        if len(self.buffer) < 4: self.buffer += "2"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn1(self):
        if len(self.buffer) < 4: self.buffer += "1"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btn0(self):
        if len(self.buffer) < 4: self.buffer += "0"; self.car_num_buffer.setPlainText(self.buffer)
    def push_btnClear(self):
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)

    # 차량 조회 함수
    def target_car_search(self):
        # 테이블위젯 초기화
        self.parked_list.clearContents()
        self.parked_list.setRowCount(0)

        if self.start_charing:  # 충전 시작 버튼을 누를 때 실행할 쿼리
            sql_query = f'''SELECT * FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND charging_start_time IS NULL
                            AND departure_time IS NULL'''
        else:                   # 충전 종료 버튼을 누를 때 실행할 쿼리
            sql_query = f'''SELECT * FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND charging_start_time IS NOT NULL
                            AND departure_time IS NULL'''

        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()

        # 조회된 차량이 없을 경우
        if not result:
            self.car_num_buffer.setPlainText("조회된 차량이 없습니다...")
            QApplication.processEvents()
            time.sleep(2)
            self.push_btnClear()
        
        # 조회되었을 경우
        else:
            for row_num, row_data in enumerate(result):
                self.parked_list.insertRow(row_num)

                for col_num, col_data in enumerate(row_data):
                    self.parked_list.setItem(row_num, col_num, QTableWidgetItem(str(col_data)))

                self.phase = 3
                self.update_kiosk_UI()

    # 초기화면에서 선택 단계로
    def phase0_clicked(self, event):
        if event.button() == 1:  # 마우스 왼쪽 버튼 클릭 확인
            self.phase = 1
        self.update_kiosk_UI()

    # 초기화면으로
    def home_btn_click(self):
        self.phase = 0
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
        self.update_kiosk_UI()

    def back_btn_click(self):
        if self.phase > 0:
            self.phase -= 1
            self.update_kiosk_UI()
        
        # 충전 종료 상태일 경우
        else:
            self.phase = 5
            self.update_kiosk_UI()

            self.target_end = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            charge_time = datetime.datetime.strptime(self.target_end, "%Y-%m-%d %H:%M:%S") - datetime.datetime.strptime(self.target_time, "%Y-%m-%d %H:%M:%S")
            
            sql_query = f'''UPDATE park_system_log
                            SET charging_end_time = '{self.target_end}',
                                price = (TIMESTAMPDIFF(SECOND, charging_start_time, '{self.target_end}') / 3600 * 3)
                            WHERE car_number = '{self.target}'
                            AND entry_time = '{self.target_time}'
                            AND departure_time IS NULL
                        '''


            remote_cursor.execute(sql_query)
            result = remote_cursor.fetchall()
            print(result)


            sql_query = f'''SELECT price FROM park_system_log
                            WHERE car_number = '{self.target}'
                            AND charging_end_time = '{self.target_end}'
                            AND departure_time IS NULL
                         '''
        
            remote_cursor.execute(sql_query)
            result = remote_cursor.fetchall()

            self.result_pay.setText(f"충전 시간은 {charge_time}분이며\n금액은 {str(result[0])}원입니다.")

            self.pay_btn.show()
            self.pay_cancel_btn.show()

    # 결제 버튼
    def pay_confirmed(self):

        self.pay_btn.hide()
        self.pay_cancel_btn.hide()

        
        remote.commit()

        self.result_pay.setText(" ****원이 결제 되었습니다.\n이용해주셔서 감사합니다.")  # 미구현!
        self.timer.start(2000)

    def auto_phase_shift(self):
        self.home_btn_click()
        self.timer.stop()

    

    # phase 변수에 따라 보여줄 UI 선택
    def update_kiosk_UI(self):
        for i, g_box in enumerate(self.groupboxes):
            if i == self.phase:
                g_box.show()
            else:
                g_box.hide()
        
        if self.phase == 1:
            self.back_btn.hide()
        else:
            self.back_btn.show()


    def push_charge_end_btn(self):
        self.phase = 2
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
        self.start_charing = False
        self.update_kiosk_UI()

    # 충전할 차량 조회 창으로
    def push_charge_start_btn(self):
        self.phase = 2
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
        self.start_charing = True
        self.update_kiosk_UI()

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

from_class2 = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/gui_4_controlPC.ui")[0]

# GUI 클래스
class ControlPCWindow(QMainWindow, from_class2):
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

        # self.arduino_timer = QTimer(self)
        # self.arduino_timer.timeout.connect(self.parking_lot_status)  # start timeout 시에 연결할 함수. 즉, 1초마다 상태값을 가져오는 함수를 호출.
        # self.arduino_timer.start(100)

        # 로봇들의 상태를 관리 및 지정하는 파트
        # self.M1 = CharingBot(1, bots_request[3], bots_status[3], 50)
        # self.M2 = CharingBot(2, bots_request[1], bots_status[2], 77)
        # self.M3 = CharingBot(3, bots_request[0], bots_status[-2], 7)
        # self.Ms = [self.M1, self.M2, self.M3]
        # self.M_labels = [self.m1_status, self.m2_status, self.m3_status]
        # self.M_tables = [self.m1_table, self.m2_table, self.m3_table]

    #     self.robot_timer = QTimer(self)
    #     self.robot_timer.timeout.connect(self.robot_status)
    #     self.robot_timer.start(100)

    #     # CCTV 설정
    #     self.isCCTVon = False
    #     self.image = None
    #     self.pixmap = QPixmap()
    #     self.cctv = CCTVCam()
    #     self.cctv.update.connect(self.updateCCTV)

    #     # 카메라 선택 구역 라디오 버튼
    #     # self.cctv_convert.clicked.connect(self.display_radio_clicked)
    #     # self.minibot1_convert.toggled.connect(self.display_radio_clicked)
    #     # self.minibot2_convert.toggled.connect(self.display_radio_clicked)
    #     # self.minibot3_convert.toggled.connect(self.display_radio_clicked)

    #     # 테이블 위젯 스케일링
    #     self.database.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m1_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m1_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m2_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m2_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m3_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    #     self.m3_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        
    #     self.DB_timer = QTimer(self)
    #     self.DB_timer.timeout.connect(self.update_parking_system_log)
    #     self.DB_timer.start(5000)

    def parking_lot_status(self, data):
        if len(data) == 3:
            print(int(data[0]), int(data[0]), int(data[1]))

        self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: lightblue;")
        self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: lightblue;")
        self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: lightblue;")
        # print(self.isOuppied)
    #     if int(serial_buffer[0]) == 0:
    #         self.B1.setText("B1"); self.B1.setStyleSheet("color: black; background-color: #f0c5c6;")

    #     if int(serial_buffer[1]) == 0:
    #         self.B2.setText("B2"); self.B2.setStyleSheet("color: black; background-color: #f0c5c6;")

    #     if int(serial_buffer[2]) == 0:
    #         self.B3.setText("B3"); self.B3.setStyleSheet("color: black; background-color: #f0c5c6;")

    #     empty_lots = int(serial_buffer[0]) + int(serial_buffer[1]) + int(serial_buffer[2])

    #     self.park_num.setText(f"{empty_lots} 대"); self.park_num.setStyleSheet("color: #f6d32d; background-color: transparent;")


    # def update_parking_system_log(self):
    #     aws_rds_access_key = import_aws_rds_access_key("/home/wintercamo/Documents/aws_rds_access_key.txt")

    #     remote = mysql.connector.connect(
    #         host = aws_rds_access_key[0],
    #         port = int(aws_rds_access_key[1]),
    #         user = aws_rds_access_key[2],
    #         password = aws_rds_access_key[3],
    #         database = aws_rds_access_key[4]
    #     )
        
    #     remote_cur = remote.cursor()
    #     remote_cur.execute("SELECT * FROM park_system_log")

    #     result = remote_cur.fetchall()

    #     # 현재 데이터베이스의 행 수와 열 수 가져오기
    #     current_rows = self.database.rowCount()
    #     current_cols = self.database.columnCount()

    #     # 데이터베이스에서 가져온 결과와 현재 테이블 위젯의 데이터 비교 및 업데이트
    #     for row_num, row_data in enumerate(result):
    #         if row_num < current_rows:
    #             # 현재 행이 있으면 업데이트
    #             for col_num, col_data in enumerate(row_data):
    #                 if col_num < current_cols:
    #                     item = self.database.item(row_num, col_num)
    #                     if item and item.text() != str(col_data):
    #                         item.setText(str(col_data))
    #                 else:
    #                     # 열 수가 현재 테이블 위젯보다 많을 경우 새로운 열 추가
    #                     if isinstance(col_data, datetime.datetime):
    #                         col_data = col_data.strftime("%d-%H:%M")
    #                         print(str(col_data))
    #                     self.database.setItem(row_num, col_num, QTableWidgetItem(str(col_data)))
    #             continue

    #         # 현재 행이 없으면 새로운 행 추가
    #         row = self.database.rowCount()
    #         self.database.insertRow(row)
    #         for col_num, col_data in enumerate(row_data):
    #             if isinstance(col_data, datetime.datetime):
    #                 col_data = col_data.strftime("%d-%H:%M")
    #                 print(str(col_data))
    #             self.database.setItem(row, col_num, QTableWidgetItem(str(col_data)))

    #     remote.close()

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
    
    # def robot_status(self):
    #     for M, label, table in zip(self.Ms, self.M_labels, self.M_tables):
    #         # bots_request = ["HUB", "B1", "B2", "B3"]
    #         # bots_status = ["사용가능", "사용불가", "이동 중", "사용 중", "충전 중", "신호없음"]

    #         if M.status == bots_status[0]:  # Hub에서 대기 중
    #             status = "사용가능"; color = "#05fa2e"
                

    #         elif M.status == bots_status[1] or M.status == bots_status[-1] or M.status == bots_status[4]:
    #             status = "사용불가"; color = "#fa0505"

    #         elif M.status == bots_status[2] or M.status == bots_status[3]:
    #             status = "사용 중"; color = "#f6fa05"

    #         label.setText(status)
    #         label.setStyleSheet(f"color: black; background-color: {color};")
        
    #         # table.setRowCount(3)
    #         # table.setColumnCount(1)
    #         table.setItem(0,0, QTableWidgetItem(str(M.request)))
    #         table.setItem(1,0, QTableWidgetItem(str(M.status)))
    #         table.setItem(2,0, QTableWidgetItem(str(M.battery_remain)))

    # def display_radio_clicked(self):
    #     if self.cctv_convert.isChecked():
    #         self.CCTVstart()
        
    #     else:
    #         self.CCTVstop()

    #     if self.minibot1_convert.isChecked():
    #         self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: black; background-color: lightgreen;")

    #     else:
    #         self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: #f6d32d;")
        
    #     if self.minibot2_convert.isChecked():
    #         self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: black; background-color: lightgreen;")

    #     else:
    #         self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: #f6d32d;")
          
    #     if self.minibot3_convert.isChecked():
    #         self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: black; background-color: lightgreen;")

    #     else:
    #         self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: #f6d32d;")
     
    # def CCTVstart(self):
    #     self.cctv.cctv_running = True
    #     self.cctv.start()
    #     self.video = cv2.VideoCapture('/dev/YourWebCam')

    # def CCTVstop(self):
    #     self.cctv.cctv_running = False
    #     self.video.release()

    # def updateCCTV(self):
    #     retval, self.image = self.video.read()
    #     if retval:
    #         self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

    #         h,w,c = self.image.shape
    #         qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

    #         self.pixmap = self.pixmap.fromImage(qimage)
    #         self.pixmap = self.pixmap.scaled(self.display.width(), self.display.height())
            
    #         self.display.setPixmap(self.pixmap)

    # def customEvent(self, event):
    #     if event.type() == ImageEvent.EVENT_TYPE:
    #         self.setImage(event.img)

    # def setImage(self, img):
    #     self.display.setPixmap(QPixmap.fromImage(img))


def main():
    app = QApplication(sys.argv)
    
    # 첫 번째 창 인스턴스 생성
    kiosk_window = KioskWindow()
    kiosk_window.show()

    # 두 번째 창 인스턴스 생성
    controlPC_window = ControlPCWindow()
    controlPC_window.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()