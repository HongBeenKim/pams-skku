import numpy as np
import sys
import os

sys.path.append(".")
from serial_packet import SerialPacket

PLANNER_FRAME_X = 1000
PLANNER_FRAME_Y = 500

MID_FRAME_X = 400
MID_FRAME_Y = 240


class Data(object):
    def __init__(self):
        self.MODES = {"default": 0, "narrow": 1, "u_turn": 2,
                      "crosswalk": 3, "target_tracking": 4,
                      "parking": 5,
                      }
        self.PARKING_MODE = {"parking_a": 6, "parking_b": 7}
        self.LIGHT_MODE = {"green_light": 8, "red_light": 9}

        self.debug_flag = False

        # from car platform
        self._read_packet = SerialPacket()
        self._read_packet.aorm = SerialPacket.AORM_MANUAL
        # to car platform
        self._write_packet = SerialPacket()

        # from sign cam
        self._detected_mission_number = self.MODES["default"]
        self._parking_lot = None  # None or 6 or 7, PARKING_MODE dict 참조
        self._light_signal = None  # None or 8 or 9, LIGHT_MODE dict 참조

        # lane cam to planner
        self.lane_value = (0, 90)  # (intercept, theta)

        # from planner
        self._current_mode = self.MODES["default"]
        self.ready_for_mission = False

        # planner to control
        self.planner_to_control_packet = (self._detected_mission_number, 300, 90, None, None)

        # monitoring
        self.sign_cam_monitoring_frame = np.zeros(shape=(MID_FRAME_Y, MID_FRAME_X, 3), dtype=np.uint8)
        self._planner_monitoring_frame = np.zeros(shape=(PLANNER_FRAME_Y, PLANNER_FRAME_X, 3), dtype=np.uint8)
        self.planner_monitoring_frame_size = (PLANNER_FRAME_X, PLANNER_FRAME_Y)

        # stop flag
        self._all_end_and_stop_yeah = False

    def stop_thinkingo(self):
        self._all_end_and_stop_yeah = True

    def is_all_system_stop(self):
        return self._all_end_and_stop_yeah

    @property
    def read_packet(self):
        return self._read_packet

    @property
    def write_packet(self):
        return self._write_packet

    def set_control_value(self, gear, speed, steer, brake):
        # gear -> 0x00: forward drive
        #         0x01: neutral
        #         0x02: backward drive
        self._write_packet.gear = gear

        # speed -> actual speed (KPH) * 10
        self._write_packet.speed = speed

        # steer -> actual steering degree (degree) * 71
        #          오차율: 4%
        #          negative is left steer
        self._write_packet.steer = steer

        # brake -> 1: no braking
        #          33: full braking
        self._write_packet.brake = brake

    def car_platform_status(self):
        gear = self._read_packet.gear
        speed = self._read_packet.speed / 10
        steer = self._read_packet.steer / 71
        brake = self._read_packet.brake / 200
        aorm = self._read_packet.aorm
        alive = self._read_packet.alive
        enc = self._read_packet.enc
        return gear, speed, steer, brake, aorm, alive, enc

    @property
    def detected_mission_number(self):
        """
        :return: a integer from 1 to 5
        """
        return self._detected_mission_number

    @detected_mission_number.setter
    def detected_mission_number(self, mission: str):
        try:
            if self.MODES[mission] != 0:
                self._detected_mission_number = self.MODES[mission]
            else:
                print("Error: detected mission number cannot be default(0)")
        except KeyError as e:
            print(e)

    @property
    def current_mode(self):
        return self._current_mode

    @current_mode.setter
    def current_mode(self, mode: int):
        self._current_mode = mode

    def reset_to_default(self):
        """
        차량 플랫폼이 manual mode 일 때 사용.
        thinkingo 시스템 전체가 표지판 대기 모드로 들어가도록 변경한다
        """
        self._detected_mission_number = self.MODES["default"]
        self._current_mode = self.MODES["default"]
        self._light_signal = None
        self._parking_lot = None

    def check_mission_completed(self):
        """
        어떤 미션이 끝나면 기본 주행으로 넘어간다.
        control.py 나 planner.py 중 미션 탈출 조건을 검사하는 곳에서 사용하는 메서드
        """
        self.reset_to_default()

    def is_in_mission(self):
        if self._current_mode == 0:
            return False
        else:
            return True

    def is_in_parking_mission(self):
        if self._current_mode == self.MODES["parking"]:
            return True
        else:
            return False

    @property
    def parking_lot(self):
        """
        :return: int 6 or 7
        """
        return self._parking_lot

    @parking_lot.setter
    def parking_lot(self, parking_location: str):
        try:
            self._parking_lot = self.PARKING_MODE[parking_location]
        except KeyError as e:
            print(e)

    @property
    def light_signal(self):
        """
        :return: int 8 or 9
        """
        return self._light_signal

    @light_signal.setter
    def light_signal(self, light: str):
        try:
            self._light_signal = self.LIGHT_MODE[light]
        except KeyError as e:
            print(e)

    def light_reset(self):
        """
        신호등을 사용하는 미션이 끝난 후 신호등 값을 None으로 리셋한다.
        control.py 가 사용하는 메서드

        """
        self._light_signal = None

    @property
    def planner_monitoring_frame(self):
        return self._planner_monitoring_frame

    @planner_monitoring_frame.setter
    def planner_monitoring_frame(self, frame_and_size: tuple):
        """
        :param frame_and_size: ([ndarray], [x-size], [y-size])
        """
        self._planner_monitoring_frame = frame_and_size[0]
        self.planner_monitoring_frame_size = (frame_and_size[1], frame_and_size[2])
