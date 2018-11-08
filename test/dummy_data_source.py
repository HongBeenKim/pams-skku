import cv2
import threading
import time

DATA_ROOT_PATH = 'c:\\pams-skku-data\\'

class DummySource():
    def __init__(self, filename: str):
        self.cap_left = cv2.VideoCapture(DATA_ROOT_PATH + 'leftcam\\' + filename + '.avi')
        self.cap_right = cv2.VideoCapture(DATA_ROOT_PATH + 'rightcam\\' + filename + '.avi')
        self.cap_mid = cv2.VideoCapture(DATA_ROOT_PATH + 'signcam\\' + filename + '.avi')
        self.lidar_file = open(DATA_ROOT_PATH + 'lidar\\' + filename + '.txt', 'rb')

        self.left_frame = None
        self.right_frame = None
        self.mid_frame = None
        self.lidar_data = None

    def data_stream(self):
        file_cursor = 0
        while True:
            #time.sleep(0.01)
            ret, self.left_frame = self.cap_left.read()
            ret, self.right_frame = self.cap_right.read()
            ret, self.mid_frame = self.cap_mid.read()

            if not ret: break

            lidar_temp = self.lidar_file.read(2500)
            lidar_temp = lidar_temp.decode()
            end_index = lidar_temp.find('')
            lidar_temp = lidar_temp[:end_index + 1]
            self.lidar_data = lidar_temp
            file_cursor += (end_index + 1)
            self.lidar_file.seek(file_cursor)

    def start(self):
        stream_thread = threading.Thread(target=self.data_stream)
        stream_thread.start()

        while self.left_frame is None or self.right_frame is None or self.mid_frame is None or self.lidar_data is None:
            pass


if __name__ == "__main__":
    import numpy as np
    RAD = 600
    dmsc = DummySource('2018-11-04-15-56-04')
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