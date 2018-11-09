import sys
import os

sys.path.append(os.path.dirname(__file__))
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
        self._read_packet = SerialPacket()
        self._write_packet = SerialPacket()
        self._detected_mission_number = 0
        self._mission_checklist = {1: False, 2: False, 3: False, 4: False, 5: False}

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

    # TODO: 주은: 준혁아 엔코더 값 필요하면 말해주렴.
    def car_platform_status(self):
        gear = self._read_packet.gear
        speed = self._read_packet.speed / 10
        steer = self._read_packet.steer / 71
        brake = self._read_packet.brake / 200
        aorm = self._read_packet.aorm
        alive = self._read_packet.alive
        return gear, speed, steer, brake, aorm, alive

    @property
    def detected_mission_number(self):
        return self._detected_mission_number

    @detected_mission_number.setter
    def detected_mission_number(self, mission: str):
        self._detected_mission_number = modes[mission]

    def check_mission_completed(self, mission: str):
        mission_num = modes[mission]
        self._mission_checklist[mission_num] = True

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
