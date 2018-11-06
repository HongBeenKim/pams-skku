import cv2
import numpy as np
import time

RAD = 500

timeLabel = "2018-11-04-16-39-23"
f = open("c:\\pams-skku-data\\lidar\\" + timeLabel + ".txt", 'rb')
cap_mid = cv2.VideoCapture("c:\\pams-skku-data\\leftcam\\" + timeLabel + ".avi")

file_cursor = 0

while True:
    current_frame = np.zeros((RAD, RAD * 2), np.uint8)
    points = np.full((361, 2), -1000, np.int)

    ret, frame = cap_mid.read()
    if not ret: break

    data_for_send = f.read(2500)
    data_for_send = data_for_send.decode()

    end_index = data_for_send.find('')
    data_for_send = data_for_send[:end_index + 1]

    file_cursor += end_index + 1
    f.seek(file_cursor)

    data_for_send = data_for_send.split(' ')[116:477]


    data_list = [int(item, 16) for item in data_for_send]
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
    cv2.imshow('test', frame)



    if cv2.waitKey(1) & 0xff == ord(' '): break

f.close()