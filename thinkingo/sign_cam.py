# -*- coding: utf-8 -*-
# TODO: 다양한 정확도로 표지판을 인식해보고 적당한 경계값 찾아내기
from module.pytorch_yolo import yolo
from subroutine import Subroutine
from data_class import Data
from data_source import Source
import time
import numpy as np

BUFFER_SIZE = 10  # 과거에 봤던 프레임을 남겨두고 한 프레임씩 밀면서 업데이트한다


class SignCam(Subroutine):
    def __init__(self, source, data: Data):
        super().__init__(data)
        self.source = source
        self.frame = None
        self.model = yolo.init_yolo_sign()
        self.ModeList = ['default', 'narrow', 'u_turn', 'crosswalk', 'target_tracking', 'parking',
                         'parking_a', 'parking_b', 'green_light', 'red_light']
        self.parking_data = [10 for row in range(BUFFER_SIZE)]
        self.traffic_data = [11 for row in range(BUFFER_SIZE)]
        self.sign_data = [0 for row in range(BUFFER_SIZE)]
        self.counter = [0 for col in range(12)]
        self.counter[0] = self.counter[10] = self.counter[11] = BUFFER_SIZE
        
    def main(self):
        while True:
            time.sleep(0.03)  # for threading schedule
            if self.source.mid_frame is None:
                continue

            if self.data.is_in_mission():
                continue
            else:
                self.frame = self.source.mid_frame.copy()
                self.data_update()
                #self.sign_selection()
                self.light_selection()
                self.parking_lot_selection()

            if self.data.is_all_system_stop():
                break

    # 하나의 데이터를 가져와 업데이트한다!
    def data_update(self):
        parking_datum, traffic_datum, sign_datum, debug_frame = yolo.run_yolo_sign(self.model, self.frame, True)
        self.data.sign_cam_monitoring_frame = debug_frame

        self.counter[self.parking_data.pop(0)] -= 1
        self.parking_data.append(parking_datum)
        self.counter[parking_datum] += 1

        self.counter[self.traffic_data.pop(0)] -= 1
        self.traffic_data.append(traffic_datum)
        self.counter[traffic_datum] += 1

        self.counter[self.sign_data.pop(0)] -= 1
        self.sign_data.append(sign_datum)
        self.counter[sign_datum] += 1

    # 어떤 표지판인지 확인하고 Data 에 넘겨주기
    def sign_selection(self):
        checkers = [0, 1, 2, 3, 4, 5]
        sign_values = []
        for i in checkers:
            sign_values.append(self.counter[i])

        max_index = np.argmax(sign_values)
        if checkers[max_index] != 0:
            self.reset_sign_buffer()
            self.data.detected_mission_number = self.ModeList[checkers[max_index]]

    # 어떤 주차장에서 주차할 것 인지 알려주기
    def parking_lot_selection(self):
        checkers = [6, 7, 10]
        parking_values = []
        for i in checkers:
            parking_values.append(self.counter[i])

        max_index = np.argmax(parking_values)

        if checkers[max_index] != 10:
            self.reset_option_buffer()
            self.data.parking_location = self.ModeList[checkers[max_index]]
            
    # 어떤 신호등인지 알려주기
    def light_selection(self):
        checkers = [8, 9, 11]
        light_values = []
        for i in checkers:
            light_values.append(self.counter[i])

        max_index = np.argmax(light_values)
        if checkers[max_index] != 11:
            self.reset_option_buffer()
            self.data.light_signal = self.ModeList[checkers[max_index]]

    def reset_sign_buffer(self):
        self.sign_data = [0 for row in range(BUFFER_SIZE)]
        self.counter[1] = self.counter[2] = self.counter[3] = self.counter[4] = self.counter[5] = 0
        self.counter[0] = BUFFER_SIZE

    def reset_option_buffer(self):
        self.parking_data = [10 for row in range(BUFFER_SIZE)]
        self.traffic_data = [11 for row in range(BUFFER_SIZE)]
        self.counter[6] = self.counter[7] = self.counter[8] = self.counter[9] = 0
        self.counter[10] = self.counter[11] = BUFFER_SIZE



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


    data_source_thread = threading.Thread(target=test_source.mid_cam_stream_main)
    sign_cam_thread = threading.Thread(target=test_sign_cam.main)

    data_source_thread.start()
    sign_cam_thread.start()
