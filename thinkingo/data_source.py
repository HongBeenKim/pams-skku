import cv2
import threading
import time
import socket

DATA_ROOT_PATH = 'c:\\pams-skku-data\\'


class Source():
    def __init__(self):
        #  웹캠 부분 (left : ),(right : ),(mid : )
        self.cap_left = cv2.VideoCapture(0)
        self.cap_right = cv2.VideoCapture(1)
        self.cap_mid = cv2.VideoCapture(2)

        self.left_frame = None
        self.right_frame = None
        self.mid_frame = None

        # 라이다 통신 연결하는 부분
        self.HOST = '169.254.248.220'
        self.PORT = 2111
        self.BUFF = 57600
        self.MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)
        self.lidar_data = None
        self.sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock_lidar.connet((self.HOST, self.PORT))
        self.sock_lidar.send(str.encode(self.MESG))
        time.sleep(2)

        # stop_flag 초기값
        self.stop_flag = False

    def video_stream(self):
        #time.sleep(0.01)
        ret, self.left_frame = self.cap_left.read()
        ret, self.right_frame = self.cap_right.read()
        ret, self.mid_frame = self.cap_mid.read()

    def lidar_stream(self):
        while True:
            lidar_datum = str(self.sock_lidar.recv(self.BUFF))
            if lidar_datum.__contains__('sEA'): continue
            temp = lidar_datum.split(' ')[116:477]
            try:
                self.lidar_data = [int(item, 16) for item in temp]
            except:
                pass

    def main(self):
        self.video_stream()
        self.lidar_stream()
        time.sleep(2)
        while True:
            self.video_stream()
            self.lidar_stream()
            while self.left_frame is None or self.right_frame is None or self.mid_frame is None or self.lidar_data is None:
                pass
            if self.stop_flag:
                self.sock_lidar.send(str.encode(chr(2) + 'sEN LMDscandata 0' + chr(3)))
                self.sock_lidar.close()
                break


if __name__ == "__main__":
    import numpy as np
    RAD = 600
    dmsc = DummySource('2018-11-04-17-01-16')
    dmsc.start()

    while True:
        current_frame = np.zeros((RAD, RAD * 2), np.uint8)
        points = np.full((361, 2), -1000, np.int)
        lidar_data = dmsc.lidar_data.split(' ')[116:477]
        data_list = [int(item, 16) for item in lidar_data]

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
            cv2.circle(current_frame, tuple(point), 20, 255, -1)  # 캔버스에 점 찍기


        cv2.imshow("LiDAR", current_frame)
        cv2.imshow('test_left', dmsc.left_frame)
        cv2.imshow('test_mid', dmsc.mid_frame)
        cv2.imshow('test_right', dmsc.right_frame)
        if cv2.waitKey(1) & 0xff == ord(' '): break

    cv2.destroyAllWindows()