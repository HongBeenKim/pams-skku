"""
서버에서 주는 라이다 데이터 받아서 단순히 그림만 그려주는 클라이언트
해당하는 시간에 함께 로깅한 영상도 같이 틀어준다.
2018-11
"""
import socket
import numpy as np
import cv2
import threading
import time

HOST = '127.0.0.1'
PORT = 10018
BUFF = 57600
timeLabel = "2018-11-04-16-24-41"
MESG = timeLabel + ".txt"
RAD = 500

flag = [False]

data_list = None

stop = False

cap_left = cv2.VideoCapture("c:\\pams-skku-data\\leftcam\\" + timeLabel + ".avi")
cap_mid = cv2.VideoCapture("c:\\pams-skku-data\\signcam\\" + timeLabel + ".avi")
cap_right = cv2.VideoCapture("c:\\pams-skku-data\\rightcam\\" + timeLabel + ".avi")

sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_lidar.connect((HOST, PORT))


def data_handling_loop():
    t_L = threading.Thread(target=video_streaming_loop, args=(flag, cap_left, "left", 0, 632))
    t_M = threading.Thread(target=video_streaming_loop, args=(flag, cap_mid, "middle", 0, 0))
    t_R = threading.Thread(target=video_streaming_loop, args=(flag, cap_right, "right", 800, 632))
    sock_lidar.send(MESG.encode())
    t_L.start()
    t_M.start()
    t_R.start()

    cv2.imshow("LiDAR", np.zeros((RAD, RAD * 2), np.uint8))
    cv2.moveWindow("LiDAR", 920, 0)

    while True:
        if flag[0]: break
        current_frame = np.zeros((RAD, RAD * 2), np.uint8)
        points = np.full((361, 2), -1000, np.int)  # 점 찍을 좌표들을 담을 어레이 (x, y), 멀리 -1000 으로 채워둠.
        data = sock_lidar.recv(BUFF).decode()

        if data.__contains__('sEA'):
            continue

        temp = data.split(' ')[116:477]

        try:
            data_list = [int(item, 16) for item in temp]
            for theta in range(0, 361):
                r = data_list[theta] / 10  # 차에서 장애물까지의 거리, 단위는 cm

                if 2 <= r:  # 라이다 바로 앞 1cm 의 노이즈는 무시

                    # r-theta 를 x-y 로 바꿔서 (실제에서의 위치, 단위는 cm)
                    x = r * np.cos(np.radians(0.5 * theta))
                    y = r * np.sin(np.radians(0.5 * theta))

                    # 좌표 변환, 화면에서 보이는 좌표(왼쪽 위가 (0, 0))에 맞춰서 집어넣는다
                    points[theta][0] = round(x) + RAD
                    points[theta][1] = RAD - round(y)
            for point in points:  # 장애물들에 대하여
                cv2.circle(current_frame, tuple(point), 2, 255, -1)  # 캔버스에 점 찍기
            cv2.imshow("LiDAR", current_frame)
            if cv2.waitKey(1) & 0xff == ord(' '): break
        except Exception:
            pass

        # TODO: 끝나면 자연스럽게 프로그램 종료하는 코드 작성하기


def video_streaming_loop(flag, cap, window_name, x, y):
    _, frame = cap.read()
    cv2.imshow(window_name, frame)
    cv2.moveWindow(window_name, x, y)
    while True:
        _, frame = cap.read()
        cv2.imshow(window_name, frame)

        if cv2.waitKey(10) & 0xff == ord(' '):  # FIXME: 싱크 안 맞음
            flag[0] = True

        if flag[0]: break

data_handling_loop()
