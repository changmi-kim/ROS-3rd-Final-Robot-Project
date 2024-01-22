import cv2
import threading
from main_server import MyServer


def print_test():
    while True:
        print(server_node.images)

# MyServer 인스턴스 생성
server_node = MyServer()


# 서버를 별도의 스레드에서 실행
server_thread = threading.Thread(target=server_node.run_server)
server_thread.start()

chat = threading.Thread(target=print_test())

