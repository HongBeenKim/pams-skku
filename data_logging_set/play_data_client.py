"""
서버에서 주는 데이터 받아서 단순히 그림만 그려주는 클라이언트
"""
import socket
import numpy as np
import cv2
import threading


HOST = ''
PORT = 10018
BUFF = 57600
timeLabel = ""
MESG = timeLabel + ".txt"
RAD = 500

data_list = None

stop = False

cap_left = cv2.VideoCapture("c:\\pams-skku-data\\leftcam\\" + timeLabel + ".avi")
cap_mid = cv2.VideoCapture("c:\\pams-skku-data\\signcam\\" + timeLabel + ".avi")
cap_right = cv2.VideoCapture("c:\\pams-skku-data\\rightcam\\" + timeLabel + ".avi")


sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_lidar.connect((HOST, PORT))


def data_handling_loop():
    t_L = threading.Thread(target=video_streaming_loop, args=(cap_left, "left"))
    t_M = threading.Thread(target=video_streaming_loop, args=(cap_mid, "middle"))
    t_R = threading.Thread(target=video_streaming_loop, args=(cap_right, "right"))

    sock_lidar.send(MESG.encode())
    t_L.start()
    t_M.start()
    t_R.start()

    while True:
        if stop: break
        data = str(sock_lidar.recv(BUFF))
        if data.__contains__('sEA'):
            continue

        temp = data.split(' ')[116:477]

        try:
            data_list = [int(item, 16) for item in temp]
        except:
            pass

        current_frame = np.zeros((RAD, RAD * 2), np.uint8)

        points = np.full((361, 2), -1000, np.int)  # 점 찍을 좌표들을 담을 어레이 (x, y), 멀리 -1000 으로 채워둠.

        for theta in range(0, 361):
            r = data_list[theta] / 10  # 차에서 장애물까지의 거리, 단위는 cm

            if 2 <= r:  # 라이다 바로 앞 1cm 의 노이즈는 무시

                # r-theta 를 x-y 로 바꿔서 (실제에서의 위치, 단위는 cm)
                x = -r * np.cos(np.radians(0.5 * theta))
                y = r * np.sin(np.radians(0.5 * theta))

                # 좌표 변환, 화면에서 보이는 좌표(왼쪽 위가 (0, 0))에 맞춰서 집어넣는다
                points[theta][0] = round(x) + RAD
                points[theta][1] = RAD - round(y)

        for point in points:  # 장애물들에 대하여
            cv2.circle(current_frame, tuple(point), 2, 255, -1)  # 캔버스에 점 찍기


def video_streaming_loop(cap, window_name):
    while True:
        _, frame = cap.read()
        cv2.imshow(window_name, frame)
        if stop:
            break

        if cv2.waitKey(1) & 0xff == ord(' '):
            stop = True


