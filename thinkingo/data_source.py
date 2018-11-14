import cv2
import socket

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
            try:
                self.lidar_data = [int(item, 16) for item in temp]
            except:
                pass
            if self.data.is_all_system_stop():
                break
        self.lidar_socket.close()
        return 0


if __name__ == "__main__":
    # TODO: 여기 적절히 채워서 테스트 해봐주세요...
    pass
