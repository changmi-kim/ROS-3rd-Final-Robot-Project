# import datetime, time
# import mysql.connector
# import random
# import string



try:
    my_query = "DROP TABLE park_system_log"
    remote_cursor.execute(my_query)
    remote.commit()

except:
    print("해당 테이블 존재 안함")

# my_query = '''CREATE TABLE park_system_log (
#                 car_number VARCHAR(16),
#                 entry_time DATETIME,
#                 charging_start_time DATETIME,
#                 charging_end_time DATETIME,
#                 departure_time DATETIME,
#                 robot_number VARCHAR(4),
#                 price FLOAT,
#                 parking_location VARCHAR(4))'''
# remote_cursor.execute(my_query)
# remote.commit()

# # 랜덤 차 번호 생성
# O_values = '0123456789'
# X_values = string.ascii_uppercase

# # 랜덤 차 번호 생성
# random_car_number = []
# for _ in range(20):
#     generated_string = ''.join([
#         random.choice(O_values) +  # 숫자
#         random.choice(O_values) +  # 숫자
#         random.choice(X_values) +  # 알파벳
#         ' ' +  # 띄어쓰기
#         random.choice(O_values) +  # 숫자
#         random.choice(O_values) +  # 숫자
#         random.choice(O_values) +  # 숫자
#         random.choice(O_values)    # 숫자
#     ])
#     random_car_number.append(generated_string)

# # 1. 입차 시각 리스트 생성 (랜덤으로 20개)
# entry_times = []
# for _ in range(20):
#     entry_time = datetime.datetime(2024, 1, 18, random.randint(0, 23), random.randint(0, 59))
#     entry_times.append(entry_time)

# # 2. 충전 시작 시각 리스트 생성 (랜덤으로 1~5분 추가, 3대는 None)
# charging_start_times = []
# for entry_time in entry_times:
#     if random.random() < 0.15:  # 15% 확률로 None 대입
#         charging_start_times.append(None)
#     else:
#         minutes_to_add = random.randint(1, 5)
#         charging_start_time = entry_time + datetime.timedelta(minutes=minutes_to_add)
#         charging_start_times.append(charging_start_time)

# # 3. 충전 완료 시각 리스트 생성 (랜덤으로 1~20분 추가, None 대입)
# charging_end_times = []
# for charging_start_time in charging_start_times:
#     if charging_start_time is None:
#         charging_end_times.append(None)
#     else:
#         if random.random() < 0.15:  # 15% 확률로 None 대입
#             charging_end_times.append(None)
#         else:
#             minutes_to_add = random.randint(1, 20)
#             charging_end_time = charging_start_time + datetime.timedelta(minutes=minutes_to_add)
#             charging_end_times.append(charging_end_time)

# # 4. 출차 시각 리스트 생성 (랜덤으로 10~20분 또는 15~30분 추가, 3대는 None)
# departure_times = []
# for charging_end_time, charging_start_time in zip(charging_end_times, charging_start_times):
#     if charging_start_time is None:
#         departure_times.append(None)
#     else:
#         if random.random() < 0.09:  # 15% 확률로 None 대입
#             departure_times.append(None)
#         else:
#             if charging_end_time is None:
#                 departure_times.append(None)
#             else:
#                 if random.random() < 0.5:  # 50% 확률로 10~20분 추가
#                     minutes_to_add = random.randint(10, 20)
#                 else:  # 50% 확률로 15~30분 추가
#                     minutes_to_add = random.randint(15, 30)
#                 departure_time = charging_end_time + datetime.timedelta(minutes=minutes_to_add)
#                 departure_times.append(departure_time)

# # 5. 금액 리스트 생성
# prices = []
# for charging_end_time, charging_start_time, departure_time, entry_time in zip(charging_end_times, charging_start_times, departure_times, entry_times):
#     if charging_end_time is None or charging_start_time is None or departure_time is None:
#         prices.append(None)
#     else:
#         charging_duration = (charging_end_time - charging_start_time).total_seconds() / 3600  # 시간 단위로 변환
#         parking_duration = (departure_time - entry_time).total_seconds() / 3600  # 시간 단위로 변환
#         price = charging_duration * 3 + parking_duration * 2
#         prices.append(price)

# # 로봇 랜덤 선택 리스트
# robot_list = ["M-1", "M-2", "M-3"]
# robot_result = []

# for i in range(20):
#     if charging_start_times[i] is None:
#         robot_result.append(None)
#     else:
#         random_robot = random.choice(robot_list)
#         robot_result.append(random_robot)

# # 주차 위치 랜덤 선택 리스트
# location_list = ["B1", "B2", "B3"]
# location_result = []

# for _ in range(20):
#     random_location = random.choice(location_list)
#     location_result.append(random_location)

# my_query = '''INSERT INTO park_system_log
#               (car_number, entry_time, charging_start_time, charging_end_time, departure_time, robot_number, price, parking_location) VALUES
#               (%s, %s, %s, %s, %s, %s, %s, %s)'''

# for car_number, entry, charging_start, charging_end, departure, robot, price, location in zip(random_car_number, entry_times, charging_start_times, charging_end_times, departure_times, robot_result, prices, location_result):
#     values = (car_number, entry, charging_start, charging_end, departure, robot, price, location)
#     remote_cursor.execute(my_query, values)

# remote.commit()
# remote.close()

import sys
from PyQt5 import uic
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time, datetime
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
remote.close()

# UI 파일 불러오기
from_class = uic.loadUiType("/home/wintercamo/gui_study/src/gui_package/gui_package/generate_test_case.ui")[0]

# GUI 클래스
class windowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Test Case Generator')

        self.fake_car_number = ""
        self.fake_entry_time = ""
        self.car_num_btn.clicked.connect(self.push_car_num_btn)

    def push_car_num_btn(self):
        self.fake_car_number = self.car_num.toPlainText()
        self.fake_entry_time = self.entry_time.toPlainText()

        if self.fake_entry_time == "":
            self.fake_entry_time = datetime.datetime.now()
        else:
            self.fake_entry_time = datetime.datetime.now() + datetime.timedelta(minutes=int(self.fake_entry_time))
        
        print(self.fake_entry_time)

        self.fake_car_number = ""; self.car_num.setText(self.fake_car_number)
        self.fake_entry_time = ""; self.entry_time.setText(self.fake_entry_time)
    
    def


def main():
    app = QApplication(sys.argv)
    window = windowClass()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()