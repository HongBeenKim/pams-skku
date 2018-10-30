"""
라이다와 캠 데이터를 동시에 저장하는 코드
"""
import threading
import cv2
import socket
import time

LIDAR_IP = '169.254.248.220'
LIDAR_PORT = 2111
LIDAR_BUF = 57600

def main():
    data = [None, None, None, False]
    camThreadL = threading.Thread(target=cam, args=(0, 1, 800, 448, data))
    camThreadR = threading.Thread(target=cam, args=(1, 2, 800, 448, data))
    lidarThread = threading.Thread(target=lidar, args=(data,))

    lidarThread.start()
    camThreadL.start()
    camThreadR.start()

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    specific_time = time.localtime()
    timeLabel = str(specific_time.tm_hour) + "+" + str(specific_time.tm_min) + "+" + str(specific_time.tm_sec)
    writerL = cv2.VideoWriter("c:\\log_data\\leftcam\\" + timeLabel + ".avi", fourcc, 30, (800, 448))
    writerR = cv2.VideoWriter("c:\\log_data\\rightcam\\" + timeLabel + ".avi", fourcc, 30, (800, 448))


    # while True:
    #     # 라이다, 캠 1, 2, 3 저장
    #     if data[0] is not None and data[1] is not None:
    #         cv2.imshow("left", data[0])
    #         cv2.imshow("right", data[1])
    #         writerL.write(data[0])
    #         writerR.write(data[1])
    #     if cv2.waitKey(1) & 0xff == ord(' '): break
    #
    # data[3] = True


def lidar(data_set: list):
    INIT_MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)
    sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_lidar.connect((LIDAR_IP, LIDAR_PORT))
    sock_lidar.send(str.encode(INIT_MESG))

    while True:
        lidar_data = str(sock_lidar.recv(LIDAR_BUF))
        if lidar_data.__contains__('sEA'): continue
        data_set[2] = lidar_data

        if data_set[3]: break


def cam(arg_num: int, cam_num: int, width: int, height: int, data_set: list):
    cap = cv2.VideoCapture(cam_num)
    cap.set(3, width); cap.set(4, height)
    while True:
        _, frame = cap.read()
        data_set[arg_num] = frame

        cv2.imshow(str(cam_num), frame)
        if cv2.waitKey(1) & 0xff == ord(' '): break

        #if data_set[3]: break

if __name__ == "__main__":
    main()
