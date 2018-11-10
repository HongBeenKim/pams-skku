import socket
import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data


class Lidar(Subroutine):
    RADIUS = 500  # 원일 경우 반지름, 사각형일 경우 한 변

    def __init__(self, data: Data):
        super().__init__(data)
        self.data_list = None
        self.frame = None
        self.stop_fg = False

        self.HOST = '169.254.248.220'
        self.PORT = 2111
        self.BUFF = 57600

        INIT_MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)
        self.lidar_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("waiting LiDAR connected. . .")
        self.lidar_socket.connect((self.HOST, self.PORT))
        print("LiDAR connect done")
        self.lidar_socket.send(str.encode(INIT_MESG))
        print("LiDAR process is waiting receive. . .")

    def main(self):  # 데이터 받아서 저장하는 메서드
        while True:
            data = str(self.lidar_socket.recv(self.BUFF))
            """
            라이다에게 데이터 요청 신호를 보냈을 때
            요청을 잘 받았다는 응답을 한 줄 받은 후에 데이터를 받기 시작함
            아래 줄은 그 응답 코드를 무시하고 바로 데이터를 받기 위해서 존재함
            """
            if data.__contains__('sEA'): continue

            temp = data.split(' ')[116:477]
            self.data.lidar_data_list = [int(item, 16) for item in temp]

            if self.stop_fg is True:
                break

        self.lidar_socket.send(str.encode(chr(2) + 'sEN LMDscandata 0' + chr(3)))
        self.lidar_socket.close()

    def stop(self):
        self.stop_fg = True


if __name__ == "__main__":
    import threading
    test_data = Data()
    current_lidar = Lidar(test_data)
    lidar_test_thread = threading.Thread(target=current_lidar.main)

    lidar_test_thread.start()