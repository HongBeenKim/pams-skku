"""
라이다 데이터(.txt)를 읽어서 TCP로 쏴 주는 코드
포트 대충 하고 호스트는 로컬로 짜셈
"""

import socket

HOST = ''
PORT = 10018
BUFFER = 57600

DATA_ROOT = 'C://pams-skku-data//lidar//'

# PLEASE GIVE ME THE INPUT .txt FILE NAME
LOG_FILE_NAME = DATA_ROOT + ''


def send_data():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print('LiDAR data serving on {}'.format(server_socket.getsockname()))

    lidar_file = open(LOG_FILE_NAME, 'r')

    client_socket, address = server_socket.accept()
    print("Client socket open, address: ", address)

    while True:
        data_for_send = lidar_file.read(BUFFER)
        client_socket.send(data_for_send.encode())

        if client_socket.recv(32) == "end":
            print("Shutdown")
            break
        else:
            pass

    server_socket.close()
