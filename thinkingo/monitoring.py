#TODO: light, parking A, B 모니터링 할 수 있도록 만들기

import cv2
import numpy as np
import datetime
import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data
from data_source import Source
from serial_packet import SerialPacket

FONT_THICKNESS = 3

PLANNER_FRAME_X = 1000
PLANNER_FRAME_Y = 500

MID_FRAME_X = 400
MID_FRAME_Y = 240

SKY_BLUE = (255, 200, 0)
YELLOW = (0, 255, 255)

DATA_ROOT_PATH = "C:\\pams-skku-monitoring-log\\2018-11-16\\"


class Monitoring(Subroutine):
    def __init__(self, source: Source, data: Data):
        super().__init__(data)
        self.source = source
        self.monitoring_size = (820, 1000)
        self.video_writer_size = (1000, 820)
        self.canvas = np.zeros(shape=(*self.monitoring_size, 3), dtype=np.uint8)
        self.mode_string = {y: x for x, y in self.data.MODES.items()}
        self.mode_string[4] = 'target'

        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        today = datetime.datetime.now()
        time_label = today.strftime("%Y-%m-%d-%H-%M-%S")
        self.monitor_writer = cv2.VideoWriter(DATA_ROOT_PATH + time_label + ".avi", fourcc, 60, self.video_writer_size)

    def main(self):
        cv2.imshow('ThinKingo monitoring', self.canvas)
        cv2.moveWindow('ThinKingo monitoring', 580, 0)
        while True:
            car_frame = self.put_car_status_and_mode()  # 240 600
            mid_cam_monitor = self.get_mid_cam_frame()  # 240 400
            planner_monitor = self.get_planner_frame()  # 500 1000

            status_frame = np.zeros((80, 1000, 3), dtype=np.uint8)
            aorm = self.data.read_packet.aorm
            if aorm == SerialPacket.AORM_MANUAL:
                aorm = 'Man'
            else:
                aorm = 'Auto'
            cv2.putText(status_frame, text=aorm, org=(0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(0, 255, 255), thickness=FONT_THICKNESS)

            write_pac = self.data.write_packet
            gear = write_pac.gear
            enc = write_pac.enc
            speed = write_pac.speed
            steer = write_pac.steer
            brake = write_pac.brake

            gear_string = ''
            if gear == SerialPacket.GEAR_FORWARD:
                gear_string = 'Drive   '
            elif gear == SerialPacket.GEAR_NEUTRAL:
                gear_string = 'Neutral '
            elif gear == SerialPacket.GEAR_BACKWARD:
                gear_string = 'Rear   '

            gear_speed_string = gear_string + '{:4.2f}'.format(speed) + 'kph'
            steer_direction_string = 'Left    ' if steer > 0 else 'Right   '
            if steer == 0:
                steer_direction_string = 'Straight'

            steer_string = steer_direction_string + '{:5.2f}'.format(abs(steer)) + 'deg'

            brake_string = 'Brake' + '{:7.2f}'.format(brake)
            status_frame = cv2.putText(img=status_frame, text=gear_speed_string, org=(150, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=1,
                                color=(255, 255, 255), thickness=FONT_THICKNESS)
            status_frame = cv2.putText(img=status_frame, text=steer_string, org=(450, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=1,
                                color=(255, 200, 200), thickness=FONT_THICKNESS)
            status_frame = cv2.putText(img=status_frame, text=brake_string, org=(750, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                                fontScale=1,
                                color=(255, 255, 255), thickness=FONT_THICKNESS)

            try:
                self.canvas = np.concatenate((car_frame, mid_cam_monitor), axis=1)
                self.canvas = np.concatenate((self.canvas, planner_monitor), axis=0)
                self.canvas = np.concatenate((self.canvas, status_frame), axis=0)
            except ValueError as e:
                print(e)
                self.init_canvas()

            cv2.imshow('ThinKingo monitoring', self.canvas)
            self.monitor_writer.write(self.canvas)
            c = cv2.waitKey(1) & 0xff
            if c == ord('0'):
                self.data.reset_to_default()
            if c == ord('1'):
                self.data.current_mode = 1
            if c == ord('2'):
                self.data.current_mode = 2
            if c == ord('3'):
                self.data.current_mode = 3
            if c == ord('4'):
                self.data.current_mode = 4
            if c == ord('5'):
                self.data.current_mode = 5
            if c == ord(' '):
                self.data.stop_thinkingo()
                break
        self.monitor_writer.release()
        cv2.destroyAllWindows()

    def init_canvas(self):
        self.canvas = np.zeros(shape=(*self.monitoring_size, 3), dtype=np.uint8)

    def get_planner_frame(self):
        planner_monitor = np.zeros(shape=(PLANNER_FRAME_Y, PLANNER_FRAME_X, 3), dtype=np.uint8)
        if self.data.planner_monitoring_frame is not None:
            planner_monitor = self.data.planner_monitoring_frame

            padding1_y = PLANNER_FRAME_Y - self.data.planner_monitoring_frame_size[1]
            padding1_x = self.data.planner_monitoring_frame_size[0]
            under_padding = np.zeros(shape=(padding1_y, padding1_x, 3), dtype=np.uint8)

            padding2_y = PLANNER_FRAME_Y
            padding2_x = PLANNER_FRAME_X - self.data.planner_monitoring_frame_size[0]
            right_padding = np.zeros(shape=(padding2_y, padding2_x, 3), dtype=np.uint8)

            try:
                planner_monitor = np.concatenate((planner_monitor, under_padding), axis=0)
                planner_monitor = np.concatenate((planner_monitor, right_padding), axis=1)
            except ValueError as e:
                print(e)
                planner_monitor = np.zeros(shape=(PLANNER_FRAME_Y, PLANNER_FRAME_X, 3), dtype=np.uint8)

        return planner_monitor

    def get_mid_cam_frame(self):
        mid_cam_monitor = np.zeros(shape=(MID_FRAME_Y, MID_FRAME_X, 3), dtype=np.uint8)
        if not self.data.is_in_mission() and self.data.sign_cam_monitoring_frame is not None:
            mid_cam_monitor = self.data.sign_cam_monitoring_frame
            mid_cam_monitor = cv2.resize(mid_cam_monitor, (MID_FRAME_X, MID_FRAME_Y))

        elif self.source.mid_frame is not None:
            mid_cam_monitor = self.source.mid_frame
            mid_cam_monitor = cv2.resize(mid_cam_monitor, (MID_FRAME_X, MID_FRAME_Y))

        return mid_cam_monitor

    def put_car_status_and_mode(self):
        frame = np.zeros((240, 600, 3), dtype=np.uint8)

        # car platform status
        gear, speed, steer, brake, aorm, alive, enc = self.data.car_platform_status()
        gear_string = ''
        if gear == SerialPacket.GEAR_FORWARD:
            gear_string = 'Drive    '
        elif gear == SerialPacket.GEAR_NEUTRAL:
            gear_string = 'Neutral  '
        elif gear == SerialPacket.GEAR_BACKWARD:
            gear_string = 'Rear    '

        gear_speed_string = gear_string + '{:4.2f}'.format(speed) + 'kph'
        steer_direction_string = 'Left    ' if steer > 0 else 'Right   '
        if steer == 0:
            steer_direction_string = 'Straight'

        steer_string = steer_direction_string + '{:5.2f}'.format(abs(steer)) + 'deg'

        brake_string = 'Brake' + '{:7.2f}'.format(brake)
        frame = cv2.putText(img=frame, text=gear_speed_string, org=(0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=steer_string, org=(0, 110), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=brake_string, org=(0, 170), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)

        # mission mode status
        current = self.mode_string[self.data.current_mode]
        detected = self.mode_string[self.data.detected_mission_number]

        frame = cv2.putText(img=frame, text=current, org=(0, 220), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=SKY_BLUE, thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=detected, org=(300, 220), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=YELLOW, thickness=FONT_THICKNESS)
        return frame


if __name__ == "__main__":
    import threading
    from data_source import Source
    from car_platform import CarPlatform
    from lane_cam import LaneCam

    test_data = Data()
    test_data_source = Source(test_data)

    # left_cam_thread = threading.Thread(target=test_data_source.left_cam_stream_main)
    # right_cam_thred = threading.Thread(target=test_data_source.right_cam_stream_main)
    mid_cam_thread = threading.Thread(target=test_data_source.mid_cam_stream_main)
    # left_cam_thread.start()
    # right_cam_thred.start()
    mid_cam_thread.start()

    car = CarPlatform('COM5', test_data)
    # lane_cam = LaneCam(test_data_source, test_data)
    monitoring = Monitoring(test_data_source, test_data)

    car_thread = threading.Thread(target=car.main)
    # lane_cam_thread = threading.Thread(target=lane_cam.main)
    monitor_thread = threading.Thread(target=monitoring.main)

    car_thread.start()
    monitor_thread.start()
