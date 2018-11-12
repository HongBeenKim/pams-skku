import sys

sys.path.append(".")

from subroutine import Subroutine
from data_class import Data

import time
import math

modes = {"default": 0, "narrow": 1, "u_turn": 2,
         "crosswalk": 3, "target_tracking": 4,
         "parking": 5,
         }


class Control(Subroutine):

    def __init__(self, data: Data):
        super().__init__(data)
        self.steer = 0  # 계산된 조향각
        self.speed = 0  # 계산된 속도
        self.gear = 0  # 계산된 기어
        self.brake = 0  # 계산된 브레이크

        self.speed_platform = 0  # 플랫폼에서 읽은 현재 속도
        self.enc_platform = 0  # 플랫폼에서 읽은 엔코더 값

        self.accel_speed = 0  # 가속된 최종 조향각
        self.accel_brake = 0  # 가속된 최종 브레이크
        self.accel_gear = 0  # 변수 형태 동일화
        self.accel_steer = 0  # 변수 형태 동일화
        #######################################
        self.mission_num = 0  # DEFAULT 모드

        self.sign_list = [0, 0, 0, 0, 0]
        #######################################
        self.steer_past = 0
        self.speed_past = 0

        ####################################### 미션용 변수
        self.change_mission = 0  # 미션 탈출용 변수
        self.u_sit = 0
        self.t_sit = 0
        self.p_sit = 0

    def main(self):
        while True:
            packet, mode_change = self.read_packet_from_planner()
            self.read_car_platform_status()
            self.do_mission(packet)
            self.accel(self.speed)
            self.write()
            pass

    def read_packet_from_planner(self):
        """
        data_class.py 를 통해 planner 에서 온 데이터 받아오기
        :return: the packet and second packet from planner
        """
        packet = self.data.planner_to_control_packet
        self.mission_num = packet[0]  # set mission number
        # TODO: @김홍빈 second 값 어떻게 줄지 약속
        return packet

    def read_car_platform_status(self):
        """
        data_class.py 에서 차량 플랫폼 데이터 받아오기
        """
        gear, speed, steer, brake, aorm, alive, enc = self.data.car_platform_status()
        self.speed_platform = speed
        self.enc_platform = enc

    def do_mission(self, packet):
        """
        :param packet:
        공통: packet[0] = detected mission number
        0. default
        0-1. second is None
            packet[1] = 차선 중앙과의 거리, 넘겨받는 단위: cm
            packet[2] = 차선함수의 기울기
        0-2. second is not None
            packet[1] = 부채꼴함수에서 계산된 거리(반지름), 넘겨받는 단위: cm
            packet[2] = 부채꼴함수에서 계산된 각도
        1. narrow
            TODO: ???
        2. u_turn
            TODO: ???
        3. crosswalk
            packet[1] = 정지선으로 부터 거리, 넘겨받는 단위: cm
        4. target_tracking
            packet[1] = target car 와의 거리, 넘겨받는 단위: cm
        5. parking
            TODO: ???
        :return: 리턴 값은 없습니다.
        """
        if self.mission_num == modes["default"]:
                if packet is None:
                    return
                self.__default__(packet[1] / 100, packet[2])
                
        elif self.mission_num == modes["narrow"]:
            self.__obs__(packet[1] / 100, packet[2])

        elif self.mission_num == modes["u_turn"]:
            self.__turn__(packet[1] / 100)
            # TODO: 수정

        elif self.mission_num == modes["crosswalk"]:
            if packet is None:
                self.__cross__(100)
            else:
                self.__cross__(packet[1] / 100)

        elif self.mission_num == modes["target_tracking"]:
            self.__target__(packet[1] / 100)

        elif self.mission_num == modes["parking"]:
            # TODO: 후에 추가로 작성하기
            return 0

    def accel(self, speed):  # TODO: 후에 실험값을 통해서 값 수정
        if self.speed_platform < ((3 * speed)/ 5):
            self.accel_speed = 2 * self.speed
            self.accel_brake = self.brake
        elif self.brake == 0 and ((self.speed_platform / 2) > self.speed) and self.speed_platform > 50:
            self.accel_speed = 0
            self.brake = 20
        else:
            self.accel_speed = self.speed
            self.accel_brake = self.brake
        self.accel_steer = self.steer
        self.accel_gear = self.gear

    def write(self):
        return self.accel_gear, self.accel_speed, self.accel_steer, self.accel_brake

    def __default__(self, cross_track_error, theta):
        gear = 0
        brake = 0

        theta_1 = (90 - theta)

        k = 1
        if abs(theta_1) < 15 and abs(cross_track_error) < 0.27:
            k = 0.5

        if self.speed_platform == 0:
            theta_2 = 0
        else:
            velocity = (self.speed_platform * 100) / 3600
            theta_2 = math.degrees(math.atan((k * cross_track_error) / velocity))

        steer_now = (theta_1 + theta_2)

        adjust = 0.3

        if abs(steer_now) > 15:
            speed = 60  # TODO: 실험값 수정하기 / 60
        else:
            speed = 72  # TODO: 실험값 수정하기 / 72

        steer_final = ((adjust * self.steer_past) + ((1 - adjust) * steer_now))
        self.steer_past = steer_final

        steer = steer_final * 71
        if steer > 1970:
            steer = 1970
            self.steer_past = 27.746
        elif steer < -1970:
            steer = -1970
            self.steer_past = -27.746

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

    def __obs__(self, obs_r, obs_theta):
        gear = 0
        brake = 0
        car_circle = 1

        correction = 1.6
        adjust = 0.05

        speed_mission = 30  # TODO: 미션용 속도, 실험하고 변경바람
        speed = speed_mission  # TODO: 연습장 상태가 좋지 않음(기울기 존재)
        if self.speed_platform > speed_mission:  # 기울기가 있어서 가속받을 경우 급정지
            speed = 0
            brake = 60

        if obs_r is None or obs_theta is None:
            print("MISSION NUMBER ERROR")
            speed = 0
            correction = 0.0
            adjust = 0.0

        self.change_mission = 1

        cal_theta = math.radians(abs(obs_theta - 90))
        cos_theta = math.cos(cal_theta)
        sin_theta = math.sin(cal_theta)

        if (90 - obs_theta) == 0:
            theta_obs = 0
        elif obs_theta == -35:
            theta_obs = 27
        elif obs_theta == -145:
            theta_obs = -27
        else:
            car_circle = 1.387
            cul_obs = (obs_r + (2.08 * cos_theta)) / (2 * sin_theta)
            theta_cal = math.atan((1.04 + (obs_r * cos_theta)) / cul_obs) / 2

            son_obs = (cul_obs * math.sin(theta_cal)) - (obs_r * cos_theta)
            mother_obs = (cul_obs * math.cos(theta_cal)) + 0.4925

            theta_obs = math.degrees(math.atan(abs(son_obs / mother_obs)))

            if abs(theta_obs) > 15:
                speed = 12
            else:
                print("OBS MODE ERROR")
                theta_obs = 0

        if (90 - obs_theta) < 0:
            theta_obs = theta_obs * (-1)

        steer_final = (adjust * self.steer_past) + (1 - adjust) * theta_obs * correction * car_circle

        self.steer_past = steer_final

        steer = steer_final * 71
        if steer > 1970:
            steer = 1970
            self.steer_past = 27.746
        elif steer < -1970:
            steer = -1970
            self.steer_past = -27.746

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

    def __turn__(self, turn_distance):
        gear = 0
        speed = 60
        steer = 0
        brake = 0

        self.change_mission = 0

        if self.u_sit == 0:
            if turn_distance < 3.50:
                speed = 30
            elif turn_distance < 3.0:
                steer = 0
                speed = 0
                brake = 65

                if self.speed_platform == 0:
                    self.u_sit = 1

            else:
                steer = 0
                speed = 60
                brake = 0
                gear = 0

        elif self.u_sit == 1:
            speed = 60
            if self.ct1 == 0:
                self.ct1 = self.enc_platform
            self.ct2 = self.enc_platform

            if (self.ct2 - self.ct1) < 630:  # TODO: 실험값 수정
                steer = -1469.79

            elif 530 <= (self.ct2 - self.ct1) <= 760:  # TODO: 실험값 수정
                speed = 30
                steer = -1469.79

            elif (self.ct2 - self.ct1) >= 760:
                steer = -1469.79
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    steer = 0
                    speed = 15
                    self.change_mission = 1

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

    def __cross__(self, stop_line):  # TODO: 신호등 값 받기
        steer = 0
        speed = 60
        gear = 0
        brake = 0

        self.change_mission = 0

        if abs(stop_line) < 1.7:  # TODO: 기준선까지의 거리값, 경로생성 알고리즘에서 값 받아오기
            return 0
        
        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

    def __target__(self, distance):
        speed = 50
        steer = 0
        gear = 0
        brake = 0

        self.change_mission = 0

        time_change = 0.5  # 값 갱신 속도, 수정바람

        if self.t_sit == 0:
            if distance < 2.0:
                speed = 30

                self.gear = gear
                self.speed = speed
                self.steer = steer
                self.brake = brake

            elif distance < 1.5:
                speed = 0
                brake = 80
                self.t_sit = 1

                self.gear = gear
                self.speed = speed
                self.steer = steer
                self.brake = brake

        elif self.t_sit == 1:
            self.speed_past = self.speed_platform
            velocity = distance - 1.5
            speed = self.speed_past + (velocity * 3.6) / time_change

            if velocity < 0:
                if distance < 1:
                    speed = 0
                    brake = 60

            if speed > 60:
                speed = 60

            if distance > 3:  # TODO: 실험값 수정
                self.change_mission = 1

            self.gear = gear
            self.speed = speed
            self.steer = steer
            self.brake = brake

    def __parking__(self):
        gear = 0
        speed = 36
        steer = 0
        brake = 0

        self.change_mission = 0

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake


# 테스트용 코드, 아래에 원하는 대로 바꿔서 테스트해볼 수 있습니다.
if __name__ == '__main__':
    import threading

    test_data = Data()
    test_control = Control(test_data)
    control_thread = threading.Thread(target=test_control.main)

    control_thread.start()
