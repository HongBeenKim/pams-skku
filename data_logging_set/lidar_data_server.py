"""
자율주행 시스템 코드에 마치 라이다를 연결한 것처럼
미리 로깅한 라이다 데이터(.txt)를 읽어서 TCP로 쏴 주는 코드
2018-10-31
"""

import socket

HOST = ''
PORT = 10018
BUFFER = 57600

DATA_PATH = 'C:\\pams-skku-data\\lidar\\'

# PLEASE GIVE ME THE INPUT .txt FILE NAME
LOG_FILE_NAME = DATA_PATH + ''
# LOG_FILE_NAME = 'C:\\Users\\Jueun\\Desktop\\sensor-data-logging-HEVEN\\data\\GLG0320_2.txt'


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


if __name__ == "__main__":
    send_data()
