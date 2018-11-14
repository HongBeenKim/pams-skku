# -*- coding: utf-8 -*-

from module.pytorch_yolo import yolo 
from subroutine import Subroutine
from data_class import Data
from data_source import Source



class SignCam(Subroutine):
    def __init__(self, source, data: Data):
        super().__init__(data)
        self.source = source
        self.model = yolo.init_yolo_sign()
        self.yolo_data = [None, None]
        self.yolo_datas = [[0 for col in range(2)] for row in range(50)]
        self.sign = [[0 for col in range(9)] for row i  n range(3)]
        self.sign_init() 

    def main(self):
        while True:
            if self.source.left_frame is None:
                continue
            if self.data.is_in_mission():
                continue

            else:
                self.frame = self.source.left_frame.copy()
                self.data_decoding()
                self.first_selection()
                self.second_selection()
                self.third_selection()
                self.parking_lot_selection()
                self.sign_reinit()
                print(self.data.detected_mission_number)

            if self.data.is_all_system_stop():
                break
        
        
        

    def sign_init(self):
        self.sign[0][0] = 'default'
        self.sign[0][1] = 'narrow'
        self.sign[0][2] = 'u_turn'
        self.sign[0][3] = 'crosswalk'
        self.sign[0][4] = 'target_tracking'
        self.sign[0][5] = 'parking'
        self.sign[0][6] = 'parking_a'
        self.sign[0][7] = 'parking_b'
        self.sign[0][8] = 'green_light'

    # 최근 몇개의 데이터를 가져와 저장한다!
    def data_decoding_init(self):
        count = 0
        while count<50:
            self.yolo_data = yolo.run_yolo_sign(self.model, self.frame)

            for data in self.yolo_data:
                self.yolo_datas[count] = data
                count += 1

    # 하나의 데이터를 가져와 업데이트한다!
    def data_decoding(self):
        self.yolo_data = yolo.run_yolo_sign(self.model, self.frame)

        for data in self.yolo_data:
            self.yolo_datas.pop(0)
            self.yolo_datas.append(data)

    #  조건1 최근 몇개의 데이터에서 특정 인식률 이상 인것에 (sign[1][#]+=1)
    def first_selection(self):
        for i in range(0, 50):
            for j in range(0, 8):
                if self.yolo_datas[i][0] == self.sign[0][j]:
                    if float(self.yolo_datas[i][1]) > 0.75:  # 최소 인식률 정하는 곳
                        self.sign[1][j] = self.sign[1][j] + 1

    #  조건2 first_selection 거친 것중에 몇회이상 나오면 (sign[2][#]=1)
    def second_selection(self):
        for i in range(0, 8):
            if self.sign[1][i] >= 25:
                self.sign[2][i] = 1

    #  조건3  이미 진행한 미션이거나 아직 수행하면 안 되는 미션인지 확인
    def third_selection(self):
        for i in range(1, 6):
            if self.sign[2][i] == 1:
                if self.data.is_next_mission(self.sign[0][i]):
                    self.data.detected_mission_number = self.sign[0][i]

    #  parking_lot 위치 정하기
    def parking_lot_selection(self):
        for i in range(6, 8):
            if self.sign[2][i] == 1:
                if self.data.is_in_parking_mission():
                    self.data.parking_location = self.sign[0][i]

    def sign_reinit(self):
        self.sign[1][0] = 0
        self.sign[1][1] = 0
        self.sign[1][2] = 0
        self.sign[1][3] = 0
        self.sign[1][4] = 0
        self.sign[1][5] = 0
        self.sign[1][6] = 0
        self.sign[1][7] = 0
        self.sign[2][0] = 0
        self.sign[2][1] = 0
        self.sign[2][2] = 0
        self.sign[2][3] = 0
        self.sign[2][4] = 0
        self.sign[2][5] = 0
        self.sign[2][6] = 0
        self.sign[2][7] = 0


if __name__ == "__main__":
    import threading


    def start_from_crosswalk(data: Data):
        for i in [1, 2]:
            data._mission_checklist[i] = True


    def start_from_u_turn(data: Data):
        data._mission_checklist[1] = True


    test_data = Data()
    test_source = Source(data=test_data)
    test_sign_cam = SignCam(test_source, data=test_data)

    """
    특정 표지판으로 시작하기 (테스트용)
    """
    # start_from_crosswalk(test_data)
    # start_from_u_turn(test_data)

    print(test_data._mission_checklist)
    
    data_source_thread = threading.Thread(target=test_source.left_cam_stream_main)
    sign_cam_thread = threading.Thread(target=test_sign_cam.main)

    data_source_thread.start()
    sign_cam_thread.start()


