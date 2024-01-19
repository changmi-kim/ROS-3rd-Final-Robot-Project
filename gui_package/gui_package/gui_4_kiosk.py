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
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/kiosk.ui")[0]

# GUI 클래스
class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Kiosk Manager')

        self.groupboxes = [self.phase0, self.phase1, self.phase2, self.phase3, self.phase4, self.phase5]
        self.phase = 0
        self.buffer = ""
        self.captured_img.setText("본인의 차량을\n선택해주세요")
        self.start_charing = True  # True: 충전시작, False: 충전종료
        self.update_kiosk_UI()

        # 관제 PC에서 가져올 정보들
        self.isBotsReday = True  # 현재 사용가능한 로봇의 유무

        self.timer = QTimer()
        self.timer.timeout.connect(self.auto_phase_shift)

        self.phase0.mousePressEvent = self.phase0_clicked
        self.charge_start_btn.clicked.connect(self.push_charge_start_btn)
        self.charge_end_btn.clicked.connect(self.push_charge_end_btn)
        self.home_btn.clicked.connect(self.home_btn_click)
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
        
        self.parked_list.itemSelectionChanged.connect(self.handle_selected_car)
        self.select_car_btn.clicked.connect(self.push_select_car_btn)
        self.pay_cancel_btn.clicked.connect(self.home_btn_click)
        self.pay_btn.clicked.connect(self.pay_confirmed)
        # 테이블 위젯 길이 조정
        self.parked_list.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.parked_list.horizontalHeader().setSectionResizeMode(0, QHeaderView.Interactive)

        # 가라용 자료
        self.parked_list.setRowCount(2)
        self.parked_list.setColumnCount(2)
        self.parked_list.setItem(0,0, QTableWidgetItem("123가1234"))
        self.parked_list.setItem(0,1, QTableWidgetItem("01M-18D-14H-55M"))
        self.parked_list.setItem(1,0, QTableWidgetItem("999나1234"))
        self.parked_list.setItem(1,1, QTableWidgetItem("01M-18D-07H-32M"))

        
    def pay_confirmed(self):
        self.result_pay.setText(" ****원이 결제 되었습니다.\n이용해주셔서 감사합니다.")  # 미구현!
        self.timer.start(2000)

    def push_select_car_btn(self):
        self.captured_img.setText("본인의 차량을\n선택해주세요")
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
        
        else:
            self.phase = 5
            self.update_kiosk_UI()
            self.result_pay.setText("충전 시간은 30분이며\n금액은 ****원입니다.")
    
    def auto_phase_shift(self):
        self.home_btn_click()
        self.timer.stop()

    def handle_selected_car(self):
        selected_items = self.parked_list.selectedItems()
        
        if selected_items:
            selected_row = selected_items[0].row()
            self.target = self.parked_list.item(selected_row, 0).text()
            
            # 미구현... 입구에서 번호판 촬영하여 S3에 업로드. 그리고 해당 차량에 맞는 이미지를 띄워야하는 부분...
            pixmap = QPixmap("/home/wintercamo/gui_study/src/gui_package/resource/SNOW_20240108_171332_671.jpg")
            self.captured_img.setPixmap(pixmap)
            self.captured_img.setPixmap(pixmap.scaled(self.captured_img.size(), Qt.KeepAspectRatio))
            self.captured_img.setAlignment(Qt.AlignCenter)

    def push_btnConfirm(self):
        if len(self.buffer) == 4:
            self.target_car_search()

    def target_car_search(self):
        # 테이블위젯 초기화
        self.parked_list.clearContents()
        self.parked_list.setRowCount(0)
        
        # 아직 주차장에 있는 차만 나오도록 쿼리 작성.

        if self.start_charing:  # 충전 시작 버튼을 누를 때 실행할 쿼리
            sql_query = f'''SELECT * FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND departure_time IS NULL'''
        else:                   # 충전 종료 버튼을 누를 때 실행할 쿼리
            sql_query = f'''SELECT * FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND departure_time IS NULL
                            AND charging_start_time IS NOT NULL'''

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

    # phase 변수에 따라 보여줄 UI 선택
    def update_kiosk_UI(self):
        for i, g_box in enumerate(self.groupboxes):
            if i == self.phase:
                g_box.show()
            else:
                g_box.hide()

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
    
    def push_charge_end_btn(self):
        self.phase = 2
        self.start_charing = False
        self.update_kiosk_UI()

    # 충전할 차량 조회 창으로
    def push_charge_start_btn(self):
        self.phase = 2
        self.start_charing = True
        self.update_kiosk_UI()

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

def main():
    app = QApplication(sys.argv)
    window = windowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
