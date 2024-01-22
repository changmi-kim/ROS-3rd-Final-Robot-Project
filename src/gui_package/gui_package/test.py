import numpy as np
import cv2
import socket
import imutils
import threading


host = "192.168.1.7"
port = 3306
client_sockets = []

video_arr = []

cap = cv2.VideoCapture(0)



try:
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_address = (host, port) 
    server_socket.bind(server_address)
    server_socket.listen()

    print("서버 생성 완료")

    while True:
        try:
            client_socket, client_address = server_socket.accept()
            client_id = str(client_address[0])
            print("client_id : ", client_id)

            client_sockets.append(client_socket)
            print(f'참여한 클라이언트 수: {len(client_sockets)}')

        except Exception as e:
            print(f'오류 발생: {e}')


finally:
    print('서버를 종료합니다.')

    for client_socket in client_sockets:
        client_socket.close()

    server_socket.close()