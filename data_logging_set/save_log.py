"""
라이다, 캠, 차량 플랫폼 데이터를 동시에 저장하는 코드
2018-11
"""
import threading
import cv2
import socket
import time
import datetime
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
from thinkingo.data_class import Data
from thinkingo.car_platform import CarPlatform

# ---------------------------- Configuration Values ---------------------------------
LIDAR_IP = '169.254.248.220'
LIDAR_PORT = 2111
LIDAR_BUF = 57600

left_cam_size = (800, 448)
right_cam_size = (800, 448)
sign_cam_size = (800, 448)

left_cam_num = 0
right_cam_num = 1
sign_cam_num = 2

COMPORT = 'COM5'

DATA_ROOT_PATH = "C:\\pams-skku-data-11-17\\"


# -----------------------------------------------------------------------------------

def main():
    platform_data = Data()
    data = [None, None, None, None, False]  # 순서대로 left frame, right frame, lidar frame, stop flag
    camThreadL = threading.Thread(target=cam, args=(0, left_cam_num, *left_cam_size, data))
    camThreadR = threading.Thread(target=cam, args=(1, right_cam_num, *right_cam_size, data))
    camThreadS = threading.Thread(target=cam, args=(2, sign_cam_num, *sign_cam_size, data))
    lidarThread = threading.Thread(target=lidar, args=(data,))
    platform_thread = threading.Thread(target=log_car_platform, args=(platform_data, data))

    lidarThread.start()
    camThreadL.start()
    camThreadR.start()
    camThreadS.start()
    platform_thread.start()

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    specific_time = time.localtime()
    today = datetime.datetime.now()
    timeLabel = today.strftime("%Y-%m-%d-%H-%M-%S")
    try:
        writerL = cv2.VideoWriter(DATA_ROOT_PATH + "leftcam\\" + timeLabel + ".avi", fourcc, 60, left_cam_size)
        writerR = cv2.VideoWriter(DATA_ROOT_PATH + "rightcam\\" + timeLabel + ".avi", fourcc, 60, right_cam_size)
        writerS = cv2.VideoWriter(DATA_ROOT_PATH + "midcam\\" + timeLabel + ".avi", fourcc, 60, sign_cam_size)
        lidar_fd = open(DATA_ROOT_PATH + "lidar\\" + timeLabel + ".txt", 'w')

        car_platform_fd = open(DATA_ROOT_PATH + "car_platform\\" + timeLabel + ".txt", 'wb')
    except FileNotFoundError as e:
        print(e)
        return 1

    while True:
        try:
            # 라이다, 캠 1, 2, 3 저장
            if data[0] is not None and data[1] is not None and data[2] is not None:
                cv2.imshow("left", data[0])
                cv2.imshow("right", data[1])
                cv2.imshow("mid", data[2])
                writerL.write(data[0])
                writerR.write(data[1])
                writerS.write(data[2])
                lidar_fd.write(data[3])
                car_platform_fd.write(platform_data.read_packet.write_bytes())
            if cv2.waitKey(1) & 0xff == ord(' '):
                break
        except Exception as e:
            print(e)
            pass

    data[4] = True
    writerL.release()
    writerR.release()
    writerS.release()
    lidar_fd.close()
    car_platform_fd.close()
    return 0


def lidar(data_set: list):
    INIT_MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)
    sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_lidar.connect((LIDAR_IP, LIDAR_PORT))
    sock_lidar.send(str.encode(INIT_MESG))

    while True:
        lidar_data = sock_lidar.recv(LIDAR_BUF).decode()
        if lidar_data.__contains__('sEA'):
            continue
        data_set[3] = lidar_data

        if data_set[4]:
            break


def cam(arg_num: int, cam_num: int, width: int, height: int, data_set: list):
    cap = cv2.VideoCapture(cam_num)
    cap.set(3, width)
    cap.set(4, height)

    while True:
        _, frame = cap.read()
        data_set[arg_num] = frame

        if data_set[4]:
            break


def log_car_platform(data: Data, data_set: list):
    platform = CarPlatform(COMPORT, data)
    while True:
        platform.receive()
        print(data.car_platform_status())
        if data_set[4]:
            break


if __name__ == "__main__":
    main()
