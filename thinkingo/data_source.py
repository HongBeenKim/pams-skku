import cv2
import time
import socket

import sys
sys.path.append(".")
from subroutine import Subroutine
from data_class import Data


class Source(Subroutine):
    def __init__(self, data: Data):
        super().__init__(data)
        # 웹캠 부분 (left : ),(right : ),(mid : )
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
        self.data.lidar_data_list = None
        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.lidar_socket.connect((self.HOST, self.PORT))
        self.lidar_socket.send(str.encode(self.MESG))
        time.sleep(2)

        # stop_flag 초기값
        self.stop_flag = False

    def video_stream(self):
        # time.sleep(0.01)
        ret, self.left_frame = self.cap_left.read()
        ret, self.right_frame = self.cap_right.read()
        ret, self.mid_frame = self.cap_mid.read()

    def lidar_stream(self):
        """
        LiDAR 데이터 받아서 저장하는 메서드
        """
        while True:
            lidar_datum = str(self.lidar_socket.recv(self.BUFF))
            if lidar_datum.__contains__('sEA'): continue
            temp = lidar_datum.split(' ')[116:477]
            try:
                self.data.lidar_data_list = [int(item, 16) for item in temp]
            except:
                pass

    def main(self):
        self.video_stream()
        self.lidar_stream()
        time.sleep(2)
        while True:
            self.video_stream()
            self.lidar_stream()
            while self.left_frame is None or self.right_frame is None \
                    or self.mid_frame is None or self.data.lidar_data_list is None:
                pass
            if self.stop_flag:
                self.lidar_socket.send(str.encode(chr(2) + 'sEN LMDscandata 0' + chr(3)))
                self.lidar_socket.close()
                break


if __name__ == "__main__":
    import numpy as np
    import threading

    RAD = 600
    test_data = Data()
    test_source = Source(test_data)
    test_source_thread = threading.Thread(target=test_source.main)
    test_source_thread.start()

    # data가 모두 들어올때까지 blocking
    while test_source.left_frame is None or test_source.right_frame is None \
            or test_source.mid_frame is None or test_data.lidar_data_list is None:
        pass

    while True:
        current_frame = np.zeros((RAD, RAD * 2), np.uint8)
        points = np.full((361, 2), -1000, np.int)
        lidar_data = test_data.lidar_data_list.split(' ')[116:477]
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
        cv2.imshow('test_left', test_source.left_frame)
        cv2.imshow('test_mid', test_source.mid_frame)
        cv2.imshow('test_right', test_source.right_frame)
        if cv2.waitKey(1) & 0xff == ord(' '): break

    cv2.destroyAllWindows()
