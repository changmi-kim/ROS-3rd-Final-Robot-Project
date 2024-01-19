import socket
from _thread import *

HOST_ = '192.168.1.7'
PORT_ = 3306


# 서버로부터 응답 받으면
def recv_data(client_socket):
    while True:
        response = client_socket.recv(1024)
        print("서버로부터 받은 응답 : ", repr(response.decode()))


# master socket 생성
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# server IP address & port
server_address = (HOST_, PORT_)

# 서버에 연결
client_socket.connect(server_address)


start_new_thread(recv_data, (client_socket,))
print("서버 연결 성공")


try:
    while True:
        # 서버에 메시지 전송
        message = input()
        
        if message == "quit":
            close_data = message
            break

        client_socket.send(message.encode())

finally:
    client_socket.close()