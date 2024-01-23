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
import pygame

# DB에 접근
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

# AWS RDS 엑세스 정보 경로
access_key = "/home/wintercamo/Documents/aws_rds_access_key.txt"

# UI 파일 불러오기
from_class1 = uic.loadUiType("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/kiosk.ui")[0]
from_class2 = uic.loadUiType("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/gui_4_controlPC.ui")[0]

# 경고음 플레이어
class Mp3Player(QThread):
    def __init__(self, file_path):
        super().__init__()
        self.file_path = file_path

    def run(self):
        # Pygame 초기화
        pygame.init()
        pygame.mixer.init()

        # MP3 파일 로드 및 재생
        pygame.mixer.music.load(self.file_path)
        pygame.mixer.music.play()

        # 음악 재생 상태 체크
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

    def stop(self):
        # 음악 종료
        pygame.mixer.music.stop()
        self.terminate()

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

# 키오스크 클래스
class KioskWindow(QMainWindow, from_class1):
    def __init__(self):
        super().__init__()
        self.setupUi(self)  # UI 설정
        self.setWindowTitle('Kiosk Manager')

        self.groupboxes = [self.phase0, self.phase1, self.phase2, self.phase3, self.phase4, self.phase5, self.phase6]
        self.connectors = [self.A1, self.A2, self.A3, self.A4]
        self.park_loc = [self.B1, self.B2, self.B3]
        self.start_charing = True  # True: 충전시작, False: 충전종료
        self.phase = 0
        self.buffer = ""
        self.captured_img.setText("본인의 차량을\n선택해주세요")
        
        self.update_kiosk_UI()

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
        # Phase 4
        self.A4_btn.clicked.connect(self.push_A4_btn)
        self.A3_btn.clicked.connect(self.push_A3_btn)
        self.A2_btn.clicked.connect(self.push_A2_btn)
        self.A1_btn.clicked.connect(self.push_A1_btn)
        self.mp3Player = Mp3Player("/home/wintercamo/git_ws/ros-repo-2/gui_package/resource/EAS.mp3")

    # 결제 요청 버튼
    def pay_confirmed(self):
        # DB 연결

        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()

        # 해당 로봇 귀환 시킴.
        # 단, 실제 커넥터에서 HUB로 복귀하는 것은 고려하지 않았기 때문에 추후에 쿼리를 수정해야함.

        sql_query = f'''SELECT robot_number from park_system_log
                        WHERE car_number = "{self.target}"
                        AND entry_time = '{self.target_entry_time}'
                        AND charging_end_time = '{self.target_charging_end}'
                        AND isPayed IS NULL
                        AND departure_time is NULL
                     '''
        
        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()
        
        # 하드웨어와 실제로 연동 안됨. 추후에 쿼리 수정해야함.
        sql_query = f'''UPDATE robot_status
                        SET status = '사용가능',
                            request = 'HUB'
                        WHERE id = {result[0][0]}
                     '''
        remote_cursor.execute(sql_query)
        
        # 결제했다고 표시
        sql_query = f'''UPDATE park_system_log
                        SET isPayed = 'Y'
                        WHERE car_number = "{self.target}"
                        AND entry_time = '{self.target_entry_time}'
                        AND charging_end_time = '{self.target_charging_end}'
                        AND isPayed IS NULL
                        AND departure_time IS NULL
                    '''
        
        remote_cursor.execute(sql_query)

        self.remote.commit()
        self.remote.close()

        self.phase = 5
        self.update_kiosk_UI()
        self.result_mention.setText(f"{self.price}원이 결제 되었습니다.\n이용해주셔서 감사합니다.")
        QApplication.processEvents()
        time.sleep(3)
        self.home_btn_click()

    def isBotsAvailable(self):
        # DB 연결
        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()

        sql_query = f'''SELECT id FROM robot_status
                        WHERE status = "사용가능"
                        ORDER BY battery_level DESC
                        LIMIT 1
                     '''
            
        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()
        result = [item[0] for item in result]

        if not result:
            # print("로봇확인",result)
            return -1
        else:
            return result[0]

    def bot_designation_results(self):
        self.bot_id = self.isBotsAvailable()
        print(self.bot_id)

        # 사용가능한 봇이 있으면
        if self.bot_id > 0:
            self.remote = create_DB_cursor(access_key)
            remote_cursor = self.remote.cursor()

            now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            
            sql_query = f'''UPDATE park_system_log
                            SET charging_start_time = '{now}',
                                robot_number = {self.bot_id},
                                connector = '{self.connector}'
                            WHERE car_number = '{self.target}'
                            AND entry_time = '{self.target_entry_time}'
                            AND departure_time IS NULL
                        '''
            remote_cursor.execute(sql_query)
            self.remote.commit()

            sql_query = f'''UPDATE robot_status
                            SET status = '사용불가',
                                request = '{self.connector}',
                                progress = '이동 중'
                                WHERE id = {self.bot_id}
                            '''
            remote_cursor.execute(sql_query)
            self.remote.commit()
            
        
            self.result_mention.setText(f"{self.target}차량 충전을 위해\n로봇이 {self.connector}에\n배정되었습니다.")
            self.remote.close()

        # 봇이 전부 사용할 수 없는 경우
        else:
            self.result_mention.setText("죄송합니다.\n현재 이용 가능한\n로봇이 없습니다.\n잠시 후에 이용해주세요")

        QApplication.processEvents()
        time.sleep(3)
        self.home_btn_click()
        

    # phase4에서 커넥터 현황을 파악하는 함수
    def select_connector(self):
        self.A1.setStyleSheet("color: black; background-color: lightgreen;")
        self.A2.setStyleSheet("color: black; background-color: lightgreen;")
        self.A3.setStyleSheet("color: black; background-color: lightgreen;")
        self.A4.setStyleSheet("color: black; background-color: lightgreen;")
        
        # DB 연결
        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()

        sql_query = f'''SELECT connector FROM park_system_log
                        WHERE charging_start_time IS NOT NULL
                        AND charging_end_time IS NULL
                        '''
    
        remote_cursor.execute(sql_query)
        self.using_connector = remote_cursor.fetchall()
        print("사용 중인 커넥터: ", self.using_connector)
        self.using_connector = [item[0] for item in self.using_connector]

        for item in self.using_connector:
            if "A1" == item:
                self.A1.setStyleSheet("color: black; background-color: pink;")
            if "A2" == item:
                self.A2.setStyleSheet("color: black; background-color: pink;")
            if "A3" == item:
                self.A3.setStyleSheet("color: black; background-color: pink;")
            if "A4" == item:
                self.A4.setStyleSheet("color: black; background-color: pink;")

        self.remote.close()
    
    # 커넥트 버튼을 눌렀을 때 이를 반영
    def update_connector(self):
        if self.connector in self.using_connector:
            # 사용 중인 것을 선택할 때 경고
            self.mp3Player.start()
            self.connetor_alert.setText("해당 커넥터는\n사용 중입니다!")
            QApplication.processEvents()
            time.sleep(2)
            self.mp3Player.stop()

        # 사용 중이 아닌 것을 선택하면 넘어감.
        else:
            self.phase = 5
            self.update_kiosk_UI()
            self.bot_designation_results()

        self.connetor_alert.setText("잘못된 커넥터를 선택하면\n충전이 되지 않습니다!")

    def push_A4_btn(self):
        self.connector = "A4"
        self.update_connector()

    def push_A3_btn(self):
        self.connector = "A3"
        self.update_connector()

    def push_A2_btn(self):
        self.connector = "A2"
        self.update_connector()

    def push_A1_btn(self):
        self.connector = "A1"
        self.update_connector()

    # phase3에서 phase4(충전시작) 혹은 phase5(충전종료)로 넘어가는 단계
    def push_select_car_btn(self):
        # 충전 시작 상태(phase4)일 경우
        if self.start_charing:
            self.phase = 4
            self.update_kiosk_UI()
            self.select_connector()
        
        # 충전 종료 상태(phase6)일 경우
        else:
            self.phase = 6
            self.update_kiosk_UI()
            # DB 연결
            self.remote = create_DB_cursor(access_key)
            remote_cursor = self.remote.cursor()
            # 일단 충전 종료 버튼을 누르면 가격과 충전 종료 시간을 갱신
            self.target_charging_end = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            charge_time = datetime.datetime.strptime(self.target_charging_end, "%Y-%m-%d %H:%M:%S") - datetime.datetime.strptime(self.target_entry_time, "%Y-%m-%d %H:%M:%S")
            
            sql_query = f'''UPDATE park_system_log
                            SET charging_end_time = '{self.target_charging_end}',
                                price = (TIMESTAMPDIFF(SECOND, charging_start_time, '{self.target_charging_end}') / 3600 * 3)
                            WHERE car_number = '{self.target}'
                            AND entry_time = '{self.target_entry_time}'
                            AND departure_time IS NULL
                        '''

            remote_cursor.execute(sql_query)
            self.remote.commit()
            # 충전 종료하는 차량의 가격을 불러오기
            sql_query = f'''SELECT price FROM park_system_log
                            WHERE car_number = '{self.target}'
                            AND entry_time = '{self.target_entry_time}'
                            AND charging_start_time IS NOT NULL
                            AND charging_end_time = '{self.target_charging_end}'
                            AND isPayed IS NULL
                            AND departure_time IS NULL
                         '''
        
            remote_cursor.execute(sql_query)
            self.price = remote_cursor.fetchall()[0]
            self.price = self.price[0]

            self.result_pay.setText(f"충전 시간은 {charge_time}분이며\n금액은 {self.price}원입니다.")
            
            self.remote.close()
        

    # phase3에서 조회된 차량 목록 중 자신의 차량을 선택하는 단게
    def handle_selected_car(self):
        self.captured_img.setText("본인의 차량을\n선택해주세요")
        selected_items = self.parked_list.selectedItems()

        if selected_items:
            selected_row = selected_items[0].row()
            self.target = self.parked_list.item(selected_row, 0).text()
            self.target_entry_time = self.parked_list.item(selected_row, 1).text()
            # 미구현... 입구에서 번호판 촬영하여 S3에 업로드. 그리고 해당 차량에 맞는 이미지를 띄워야하는 부분...
            pixmap = QPixmap("/home/wintercamo/gui_study/src/gui_package/resource/SNOW_20240108_171332_671.jpg")
            self.captured_img.setPixmap(pixmap)
            self.captured_img.setPixmap(pixmap.scaled(self.captured_img.size(), Qt.KeepAspectRatio))
            self.captured_img.setAlignment(Qt.AlignCenter)

    # phase 3로 넘어가기 위한 단계.
    def target_car_search(self):
        # DB 연결
        self.remote = create_DB_cursor(access_key)
        remote_cursor = self.remote.cursor()
        # 테이블위젯 초기화
        self.parked_list.clearContents()
        self.parked_list.setRowCount(0)

        # 충전 시작일 경우? 주차장에 들어와있고, 출차를 안했으며, 충전 시작을 안한 차량.
        if self.start_charing:  
            sql_query = f'''SELECT car_number, entry_time FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND entry_time IS NOT NULL
                            AND charging_start_time IS NULL
                            AND departure_time IS NULL'''
        # 충전 종료일 경우? 주차장에 들어와있고, 출차를 안했으며, 충전 시작을 한 차량.
        else:
            sql_query = f'''SELECT car_number, entry_time FROM park_system_log WHERE RIGHT(car_number, 4) = "{self.buffer}"
                            AND entry_time IS NOT NULL
                            AND charging_start_time IS NOT NULL
                            AND isPayed IS NULL
                            AND departure_time IS NULL'''
            
        remote_cursor.execute(sql_query)
        result = remote_cursor.fetchall()

        # 조회된 차량이 없을 경우
        if not result:
            self.mp3Player.start()
            self.car_num_buffer.setPlainText("조회된 차량이 없습니다...")
            QApplication.processEvents()
            time.sleep(2)
            self.mp3Player.stop()
            self.push_btnClear()
        
        # 조회되었을 경우
        else:
            for row_num, row_data in enumerate(result):
                self.parked_list.insertRow(row_num)

                for col_num, col_data in enumerate(row_data):
                    self.parked_list.setItem(row_num, col_num, QTableWidgetItem(str(col_data)))

            self.phase = 3
            self.update_kiosk_UI()
        
        self.remote.close()

    # phase2에서 phase 3로 넘어가는 단계
    def push_btnConfirm(self):
        if len(self.buffer) == 4:  # 차량번호 뒤 4자리를 전부 입력할 때만 반응
            self.target_car_search()

    # phase 1에서 phase2로 넘어와서 차량번호를 선택하는 단계
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
    
    # phase1에서 충전 종료 버튼을 눌렀을 때
    def push_charge_end_btn(self):
        self.phase = 2
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
        self.start_charing = False
        self.update_kiosk_UI()

    # phase1에서 충전 시작 버튼을 눌렀을 때
    def push_charge_start_btn(self):
        self.phase = 2
        self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
        self.start_charing = True
        self.update_kiosk_UI()
    
    # 홈 화면 가기 버튼
    def home_btn_click(self):
        self.phase = 0
        try:
            self.remote.close()
        except:
            print("rds 연결끊긴 상태")
        finally:
            self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
            self.update_kiosk_UI()
    
    # 뒤로가기 버튼
    def back_btn_click(self):
        if self.phase > 0:
            self.phase -= 1
            self.update_kiosk_UI()
        
        try:
            self.remote.close()
        except:
            print("rds 연결끊긴 상태")
        finally:
            self.buffer = ""; self.car_num_buffer.setPlainText(self.buffer)
            self.update_kiosk_UI()

    # phase0에서 phase1(충전 시작/종료 선택)로
    def phase0_clicked(self, event):
        if event.button() == 1:  # 마우스 왼쪽 버튼 클릭 확인
            self.phase = 1
        self.update_kiosk_UI()

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

# AWS RDS 연결
class MySQL():
    def __init__(self):
        self.host = os.environ["DB_address"]
        self.port = int(os.environ["DB_port"])
        self.user = os.environ["admin_id"]
        self.password = os.environ["DB_password"]
        self.database = os.environ["DB_name"]

    async def connectDB(self):
        self.remote = await aiomysql.connect(
                      host = self.host,
                      port = self.port,
                      user = self.user,
                      password = self.password,
                      db = self.database
                      )

    async def serchDB(self, command):
        if not hasattr(self, 'remote'):
            await self.connectDB()
        
        async with self.remote.cursor() as cur:
            await cur.execute(command)

            result = await cur.fetchall()
            return result

    async def disconnectDB(self):
        await self.remote.commit()
        await self.remote.close()

# 고객의 주차 여부를 파악하기 위한 Arduino 시리얼을 받아오는 스레드 클래스
class ArduinoSerial(QThread):
    receive = pyqtSignal(str)

    def __init__(self, serial=None):
        super().__init__()
        self.serial = serial
        self.running = True

    def run(self):
        while self.running == True:
            try:
                if self.serial.readable():
                    serial_buffer = self.serial.readline().decode('utf-8').strip()
                    self.receive.emit(serial_buffer)
            except:
                print('Arduino Waiting...')

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

class SubscriberImage(QThread):
    changePixmap = pyqtSignal(QImage, np.ndarray)
    flag = True
    client_socket = None

    def run(self):
        while self.flag:
            length = self.recvall(self.client_socket, 16)

            string_data = self.recvall(self.client_socket, int(length))
            data = np.frombuffer(string_data, dtype='uint8')
            frame = cv2.imdecode(data, cv2.IMREAD_COLOR)

            h, w, c = frame.shape
            qimage = QImage(frame.data, w, h, w*c, QImage.Format_RGB888)

            self.changePixmap.emit(qimage.scaled(200, 200), frame)


    def recvall(self, sock, count):
        # 바이트 문자열
        buf = b''

        while count:
            try:                
                newbuf = sock.recv(count)
                print("반복문 진입")

                if not newbuf:
                    return None
                
                buf += newbuf
                count -= len(newbuf)

            except Exception as e:
                print(e)
                break

        return buf

from_class2 = uic.loadUiType("/home/wintercamo/git_ws/ros-repo-2/gui_package/gui_package/gui_4_controlPC.ui")[0]

# 관제 PC GUI
class ControlPCWindow(QMainWindow, from_class2):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('Voltie Manager')

        self.subcriber_ = SubscriberImage(self)
        self.client_sockets = {}

        self.timer = QTimer(self)
        self.HOST = os.environ["HOST_IP"]
        self.PORT = os.environ["HOST_PORT"]


        self.images = {}  # 클라이언트별 최신 이미지를 저장하는 딕셔너리

        self.ip_name = {"192.168.1.14" : "minibot1",
                        "192.168.1.7" : "minibot2",
                        "192.168.1.6" : "minibot3",
                        }
        
        # 각 스레드에서 서버 실행
        server_thread = threading.Thread(target=self.run_server)
        server_thread.start()

        # 주자창에 주차된 차를 파악하기 위한 MCU 설정
        arduino_port = os.environ["Arduino_port"]
        baud_rate = int(os.environ["Baud_rate"])

        self.arduino_conn = Serial(port=arduino_port, baudrate=baud_rate)
        self.arduino = ArduinoSerial(self.arduino_conn)
        self.arduino.receive.connect(self.parking_lot_status)
        self.arduino.start()
        self.isOuppied = []

        # 로봇들의 상태를 관리 및 지정하는 파트
        self.M_status = [self.m1_status, self.m2_status, self.m3_status]
        self.M_tables = [self.m1_table, self.m2_table, self.m3_table]

        self.bot_timer = QTimer(self)
        self.bot_timer.timeout.connect(lambda: asyncio.create_task(self.bots_status()))
        self.bot_timer.start(100)

        # CCTV 설정
        self.isCCTVon = False
        self.image = None
        self.cctv_pixmap = QPixmap()

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
                
        self.park_DB_timer = QTimer(self)
        self.park_DB_timer.timeout.connect(lambda: asyncio.create_task(self.update_parking_system_log()))
        self.park_DB_timer.start(2000)
        
    # DB 정보 가져오기
    async def update_parking_system_log(self):
        # self.database.clearContents()
        # self.database.setRowCount(0)
        # result = await self.remote.serchDB('''SELECT car_number, entry_time, charging_start_time, charging_end_time, departure_time, robot_number, price, isPayed, connector
        #                                       FROM park_system_log''')

        # for id_item in result:
        #     row = self.database.rowCount()
        #     self.database.insertRow(row)
        #     for col_num, col_data in enumerate(id_item):
        #         print(id_item, col_num, col_data)
        #         if isinstance(col_data, datetime.datetime):
        #             col_data = col_data.strftime("%d-%H:%M")
        #         self.database.setItem(row, col_num, QTableWidgetItem(str(col_data)))
        result = await self.remote.serchDB('''SELECT car_number, entry_time, charging_start_time, charging_end_time, departure_time, robot_number, price, isPayed, connector
                                          FROM park_system_log''')

        current_row_count = self.database.rowCount()  # 현재 행 수 가져오기

        # 현재 데이터베이스 행 수와 결과 행 수 비교
        if current_row_count < len(result):
            # 결과 행이 더 많을 경우, 새로운 행 추가
            for id_item in result[current_row_count:]:
                row = self.database.rowCount()
                self.database.insertRow(row)
                for col_num, col_data in enumerate(id_item):
                    if isinstance(col_data, datetime.datetime):
                        col_data = col_data.strftime("%Y-%m-%d %H:%M:%S")
                    self.database.setItem(row, col_num, QTableWidgetItem(str(col_data)))
        
        # 결과 행 수만큼 반복하면서 데이터베이스 업데이트
        for row_num, id_item in enumerate(result):
            for col_num, col_data in enumerate(id_item):
                if isinstance(col_data, datetime.datetime):
                    col_data = col_data.strftime("%Y-%m-%d %H:%M:%S")
                current_item = self.database.item(row_num, col_num)
                if current_item and current_item.text() != str(col_data):
                    current_item.setText(str(col_data))

    def display_radio_clicked(self):
        # 라디오 버튼 체크 항목 반영 전 초기화
        if self.isCCTVon == True:
            self.CCTVstop()

        self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: #f6d32d;")
        self.m2_name.setText("M-2"); self.m2_name.setStyleSheet("color: #f6d32d;")
        self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: #f6d32d;")

        # CCTV 라디오가 체크 될 경우
        if self.cctv_convert.isChecked():

            self.cctv = CCTVCam()
            self.cctv.update.connect(self.updateCCTV)
            self.CCTVstart()

            self.display.show()
            self.minibot1_display.hide()
            self.minibot2_display.hide()
            self.minibot3_display.hide()

        # 로봇1이 체크 될 경우
        elif self.minibot1_convert.isChecked():
            self.m1_name.setText("M-1"); self.m1_name.setStyleSheet("color: black; background-color: lightgreen;")

            self.display.hide()
            self.minibot1_display.show()
            self.minibot2_display.hide()
            self.minibot3_display.hide()

            
        # 로봇2가 체크 될 경우
        elif self.minibot2_convert.isChecked():
            self.m2_name.setText("M-2")
            self.m2_name.setStyleSheet("color: black; background-color: lightgreen;")

            self.display.hide()
            self.minibot1_display.hide()
            self.minibot2_display.show()
            self.minibot3_display.hide()

        # 로봇3가 체크 될 경우  
        elif self.minibot3_convert.isChecked():
            self.m3_name.setText("M-3"); self.m3_name.setStyleSheet("color: black; background-color: lightgreen;")

            self.display.hide()
            self.minibot1_display.hide()
            self.minibot2_display.hide()
            self.minibot3_display.show()

    def setImage(self, image):
        self.minibot2_display.setPixmap(QPixmap.fromImage(image))

    async def bots_status(self):
        self.remote = MySQL()
        await self.remote.connectDB()
        result = await self.remote.serchDB("SELECT * FROM robot_status")

        
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

    def run_server(self):
        print(f'Server Start with IP: {self.HOST}')
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = (self.HOST, int(self.PORT))
        self.server_socket.bind(server_address)
        self.server_socket.listen()

        print("Server is listening...")

        try:
            while True:
                client_socket, client_address = self.server_socket.accept()
                client_id = str(client_address[0])

                if client_address[0] in self.ip_name:
                    print(f'연결 수락됨: {self.ip_name[client_address[0]]}')

                else:
                    print("미등록 사용자입니다. : ", client_address[0])
                    
                print("client_id : ", client_id)

                self.client_sockets[client_id] = client_socket
                response = '서버 연결 성공'
                client_socket.sendall(response.encode())

        except KeyboardInterrupt:
            print('서버를 종료합니다.')

            for client_socket in self.client_sockets:
                client_socket.close()

            self.server_socket.close()
            
        finally:
            # 어떠한 경우에도 마지막에 서버 소켓을 닫음
            if self.server_socket:
                self.remote.disconnectDB()
                self.server_socket.close()
    
    def parking_lot_status(self, data):
        if len(data) == 4:
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
            try:
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

                h,w,c = self.image.shape
                qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

                self.cctv_pixmap = self.cctv_pixmap.fromImage(qimage)
                self.cctv_pixmap = self.cctv_pixmap.scaled(self.display.width(), self.display.height())
                
                self.display.setPixmap(self.cctv_pixmap)

            except:
                print("cctv 에러")

async def main():
    load_dotenv()

    app = QApplication(sys.argv)

    kiosk_window = KioskWindow()
    kiosk_window.show()

    control_PC_window = ControlPCWindow()
    control_PC_window.show()

    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    with loop:
        loop.run_forever()

if __name__ == '__main__':
    asyncio.run(main())