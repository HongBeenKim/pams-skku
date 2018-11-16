import numpy as np
import sys

sys.path.append(".")
from serial_packet import SerialPacket

NUM_OF_MISSION = 5  # 기본 주행 제외한 개수

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

        self.start_from_main_flag = False

        # from car platform
        self._read_packet = SerialPacket()
        # to car platform
        self._write_packet = SerialPacket()

        # from sign cam
        # TODO: 표지판 안 될때의 보험 쓸지 말지 결정하기
        self._mission_checklist = {1: False, 2: False, 3: False, 4: False, 5: False}  # 보험용
        self._detected_mission_number = self.MODES["default"]
        self._parking_lot = None  # None or 6 or 7, PARKING_MODE dict 참조
        self._light_signal = None  # None or 8 or 9, LIGHT_MODE dict 참조

        # lane cam to planner
        self.lane_value = (0, 90)  # (intercept, theta)

        # from planner
        self._current_mode = self.MODES["default"]

        # planner to control
        self.planner_to_control_packet = (self._detected_mission_number, 300, 90, None)

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

    def check_mission_completed(self, mission: str):
        """
        어떤 미션이 끝나면 그 미션을 수행했다고 체크한 뒤 기본 주행으로 넘어간다.
        # control.py 가 사용하는 메서드 TODO: @박준혁 컨트롤에서 미션을 끝내면 이 메서드로 체크하도록 수정하기
        :param mission: 미션 이름 string (self.modes dictionary 참조)
        """
        try:
            mission_num = self.MODES[mission]
            self._mission_checklist[mission_num] = True
            self._detected_mission_number = self.MODES["default"]
            self._current_mode = self.MODES["default"]
        except KeyError as e:
            print(e)

    def check_u_turn_complete(self):
        """
        유턴을 끝내고 나면 다음 미션인 횡단보도로 넘어간다. (보험용)
        """
        self._mission_checklist[self.MODES["u_turn"]] = True
        self._detected_mission_number = self.MODES["crosswalk"]

    def check_crosswalk_complete(self):
        """
        횡단보도 미션을 끝내고 나면 다음 미션인 타겟차 트래킹으로 넘어간다. (보험용)
        """
        self._mission_checklist[self.MODES["crosswalk"]] = True
        self._detected_mission_number = self.MODES["target_tracking"]

    def is_next_mission(self, mission: str):
        try:
            if self.MODES[mission] == self.MODES["default"]:
                return True
            result = False
            for num, okay in self._mission_checklist.items():
                if okay:
                    continue
                else:
                    if num == self.MODES[mission]:
                        result = True
                        break
                    else:
                        result = False
                        break
            return result
        except KeyError as e:  # dictionary not has such key
            print(e)
            return False

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
        TODO: @박준혁 신호등 보는 미션 후 light_reset() 메서드로 리셋하도록 수정하기
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
