import cv2
import socket
import numpy as np
import math
import sys

sys.path.append(".")
from data_class import Data


class Source():
    HOST = '169.254.248.220'
    PORT = 2111
    BUFF = 57600
    MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)

    def __init__(self, data: Data):
        self.data = data
        # 웹캠 부분 (left : ),(right : ),(mid : )
        self.cap_left = cv2.VideoCapture(0)
        self.cap_right = cv2.VideoCapture(1)
        self.cap_mid = cv2.VideoCapture(2)

        self.cap_left.set(3, 800)
        self.cap_left.set(4, 448)

        self.cap_right.set(3, 800)
        self.cap_right.set(4, 448)

        self.cap_mid.set(3, 800)
        self.cap_mid.set(4, 448)

        self.left_frame = None
        self.right_frame = None
        self.mid_frame = None
        self.lidar_data = None

        self.lidar_init_fail_flag = False
        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        print("waiting LiDAR connected. . .")
        try:
            self.lidar_socket.connect((self.HOST, self.PORT))
            print("LiDAR connect complete!")
            self.lidar_socket.send(str.encode(self.MESG))
        except OSError as e:
            print(e)
            self.lidar_init_fail_flag = True

    def left_cam_stream_main(self):
        while True:
            _, self.left_frame = self.cap_left.read()

            if self.data.is_all_system_stop():
                break
        self.cap_left.release()

    def right_cam_stream_main(self):
        while True:
            _, self.right_frame = self.cap_right.read()
            if self.data.is_all_system_stop():
                break
        self.cap_right.release()

    def mid_cam_stream_main(self):
        while True:
            _, self.mid_frame = self.cap_mid.read()
            if self.data.is_all_system_stop():
                break
        self.cap_mid.release()

    def lidar_stream_main(self):
        if self.lidar_init_fail_flag:
            return 1
        while True:
            raw_data = str(self.lidar_socket.recv(self.BUFF))
            if raw_data.__contains__('sEA'): continue
            temp = raw_data.split(' ')[116:477]
            self.lidar_data = [1000000] * 361
            try:
                for i in range(0, 361):
                    r = int(temp[i], 16)
                    if r > 3: self.lidar_data[i] = r
            except:
                pass
            if self.data.is_all_system_stop():
                break
        self.lidar_socket.close()
        return 0

    def get_lidar_ndarray_data(self, y_pixel_size, x_pixel_size, point_size):
        #  모든 거리 값을 좌표로 변환해 점찍기 (왼쪽 상단 0, 0으로!)
        lidar_mat = np.zeros((y_pixel_size + 1, x_pixel_size + 1, 3), dtype=np.uint8)

        for i in range(len(self.lidar_data)):
            radian_degree = math.radians(i / 2)  # 라디안으로 바꾼 각도

            x_coordinate = int(x_pixel_size / 2) + int((self.lidar_data[i] / 10) * math.cos(radian_degree))
            y_coordinate = int(y_pixel_size) - int((self.lidar_data[i] / 10) * math.sin(radian_degree))
            cv2.circle(lidar_mat, (x_coordinate, y_coordinate), point_size, (255, 255, 255), -1)
        return lidar_mat


if __name__ == "__main__":
    pass
