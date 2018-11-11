import sys

sys.path.append(".")
from serial_packet import SerialPacket

modes = {"default": 0, "narrow": 1, "u_turn": 2,
         "crosswalk": 3, "target_tracking": 4,
         "parking": 5,
         }
parking_mode = {"parking_a": 6, "parking_b": 7}
light_mode = {"green_light": 8}
NUM_OF_MISSION = 5  # 기본 주행 제외한 개수


class Data(object):
    def __init__(self):
        # car platform
        self._read_packet = SerialPacket()
        self._write_packet = SerialPacket()

        # sign cam
        self._detected_mission_number = 0
        self._mission_checklist = {1: False, 2: False, 3: False, 4: False, 5: False}
        self._parking_lot = None  # None or 6 or 7, parking_mode dict 참조

        # data source
        self._lidar_data_list = None

        # planner
        self._motion_parameter = None

        # lane cam
        self.intercept = 0
        self.theta = 0

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
        return self._detected_mission_number

    @detected_mission_number.setter
    def detected_mission_number(self, mission: str):
        self._detected_mission_number = modes[mission]

    def check_mission_completed(self, mission: str):
        """
        어떤 미션이 끝나면 그 미션을 수행했다고 체크한 뒤 기본 주행으로 넘어간다.
        :param mission: 미션 이름 string (modes dictionary 참조)
        """
        mission_num = modes[mission]
        self._mission_checklist[mission_num] = True
        self._detected_mission_number = modes["default"]

    def check_u_turn_complete(self):
        """
        유턴을 끝내고 나면 다음 미션인 횡단보도로 넘어간다.
        """
        self._mission_checklist[modes["u_turn"]] = True
        self._detected_mission_number = modes["crosswalk"]

    def check_crosswalk_complete(self):
        """
        횡단보도 미션을 끝내고 나면 다음 미션인 타겟차 트래킹으로 넘어간다.
        """
        self._mission_checklist[modes["crosswalk"]] = True
        self._detected_mission_number = modes["target_tracking"]

    def is_next_mission(self, mission: str):
        result = False
        for num, okay in self._mission_checklist.items():
            if okay:
                continue
            else:
                if num == modes[mission]:
                    result = True
                    break
                else:
                    result = False
                    break
        return result

    def is_in_mission(self):
        if self._detected_mission_number == 0:
            return False
        else:
            return True

    def is_in_parking_mission(self):
        if self._detected_mission_number == modes["parking"]:
            return True
        else:
            return False

    @property
    def parking_lot(self):
        return self._parking_lot

    @parking_lot.setter
    def parking_lot(self, parking_location: str):
        self._parking_lot = parking_mode[parking_location]

    @property
    def lidar_data_list(self):
        return self._lidar_data_list

    @lidar_data_list.setter
    def lidar_data_list(self, lidar_data: list):
        self._lidar_data_list = lidar_data

    @property
    def motion_parameter(self):
        return self._motion_parameter

    @motion_parameter.setter
    def motion_parameter(self, parameter: tuple):
        self._motion_parameter = parameter
