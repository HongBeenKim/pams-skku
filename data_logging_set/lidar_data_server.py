"""
자율주행 시스템 코드에 마치 라이다를 연결한 것처럼
미리 로깅한 라이다 데이터(.txt)를 읽어서 TCP로 쏴 주는 코드
2018-11
"""

import socket
import time

HOST = ''
PORT = 10018
BUFFER = 57600

DATA_PATH = 'C:\\pams-skku-data\\lidar\\'


def send_data():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(1)
    print('LiDAR data serving on {}'.format(server_socket.getsockname()))

    client_socket, address = server_socket.accept()

    txt_file_name = client_socket.recv(64)
    txt_file_name = txt_file_name.decode()
    print("txt file name: ", txt_file_name)
    # PLEASE GIVE ME THE INPUT .txt FILE NAME
    LOG_FILE_NAME = DATA_PATH + txt_file_name
    lidar_file = open(LOG_FILE_NAME, 'r')

    stop_flag = False
    while True:
        data_for_send = lidar_file.read(BUFFER)
        client_socket.send(data_for_send.encode())
        time.sleep(0.5)  # FIXME: 싱크 안 맞음. 어떻게 맞추는 거지?

        if stop_flag:
            print("Shutdown")
            break
        else:
            pass

    server_socket.close()


if __name__ == "__main__":
    send_data()
