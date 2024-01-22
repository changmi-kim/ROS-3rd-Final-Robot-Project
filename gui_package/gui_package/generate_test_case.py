import sys
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import datetime
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

aws_rds_access_key = import_aws_rds_access_key("/home/wintercamo/Documents/aws_rds_access_key.txt")

remote = mysql.connector.connect(
    host = aws_rds_access_key[0],
    port = int(aws_rds_access_key[1]),
    user = aws_rds_access_key[2],
    password = aws_rds_access_key[3],
    database = aws_rds_access_key[4]
)

remote_cursor = remote.cursor()
my_sql = open("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/table_park_system_log.sql").read()

try:
    remote_cursor.execute(my_sql)
    remote.commit()

except:
    print("주차장 테이블 이미 존재함")

remote_cursor = remote.cursor()
my_sql = open("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/table_robot_status.sql").read()

try:
    remote_cursor.execute(my_sql)
    remote.commit()
    
    my_sql = '''INSERT INTO robot_status
                (status, request, progress, battery_level)
                VALUES ('사용가능', 'HUB', '도착', 100)
             '''
    
    for _ in range(3):
        remote_cursor.execute(my_sql)
        remote.commit()

except:
    print("로봇 테이블 이미 존재함")

# UI 파일 불러오기
from_class = uic.loadUiType("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/generate_test_case.ui")[0]

# GUI 클래스
class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Test Case Generator')

        self.fake_car_number = ""
        self.fake_entry_time = ""
        self.fake_departure_time = ""
        self.car_num_btn.clicked.connect(self.push_car_num_btn)
        self.depart_time_btn.clicked.connect(self.push_depart_time_btn)
        self.bye_btn.clicked.connect(self.push_bye_btn)

    def push_bye_btn(self):
        print("rds 연결 종료됨")
        remote.close()

    def push_car_num_btn(self):
        self.fake_car_number = self.car_num.toPlainText()
        self.fake_entry_time = self.entry_time.toPlainText()

        if self.fake_entry_time == "":
            self.fake_entry_time = datetime.datetime.now()
        else:
            self.fake_entry_time = datetime.datetime.now() + datetime.timedelta(minutes=int(self.fake_entry_time))
        
        self.fake_entry_time = self.fake_entry_time.strftime("%Y-%m-%d %H:%M:%S")
        print(self.fake_entry_time)
        my_query = "INSERT INTO park_system_log (car_number, entry_time) VALUES (%s, %s)"
        values = (str(self.fake_car_number), str(self.fake_entry_time),)
        remote_cursor.execute(my_query, values)
        remote.commit()

        self.fake_car_number = ""; self.car_num.setText(self.fake_car_number)
        self.fake_entry_time = ""; self.entry_time.setText(self.fake_entry_time)
    
    def push_depart_time_btn(self):
        self.fake_car_number = self.car_num.toPlainText()
        self.fake_departure_time = self.depart_time.toPlainText()

        if self.fake_departure_time == "":
            self.fake_departure_time = datetime.datetime.now()
        else:
            self.fake_departure_time = datetime.datetime.now() + datetime.timedelta(minutes=int(self.fake_departure_time))
        
        self.fake_departure_time = self.fake_departure_time.strftime("%Y-%m-%d %H:%M:%S")

        print(self.fake_departure_time)
        my_query = f'''UPDATE park_system_log
                       SET departure_time = %s
                       WHERE car_number = '{self.fake_car_number}'
                    '''
        values = (str(self.fake_departure_time),)
        remote_cursor.execute(my_query, values)
        remote.commit()

        self.fake_car_number = ""; self.car_num.setText(self.fake_car_number)
        self.fake_departure_time = ""; self.depart_time.setText(self.fake_entry_time)

def main():
    app = QApplication(sys.argv)
    window = windowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()