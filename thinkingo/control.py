import sys

sys.path.append(".")

from subroutine import Subroutine
from data_class import Data

import time
import math


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

        ####################################### 미션용 변수
        self.change_mission = 0  # 미션 탈출용 변수
        self.u_sit = 0
        self.t_sit = 0
        self.p_sit = 0

    def main(self):
        """
        차량 제어 코드가 thinkingo/main.py 에서
        thread 로 실행될 때 한 루프에서 해야 할
        모든 행동을 이 메서드에 정의합니다.
        """
        while True:
            # TODO: @박준혁 이곳에 작성하세요!
            pass

    def read(self):
        """
        data_class.py 에서 데이터 받아오기
        """
        gear, speed, steer, brake, aorm, alive, enc = self.data.car_platform_status()
        self.speed_platform = speed
        self.enc_platform = enc

    def mission(self):
        """
        판단 함수로부터 필요한 값 받기
        """
        mission_num = self.data.detected_mission_number
        first, second = self.data  # TODO: 나중에 수정하기
        self.set_mission(mission_num)
        self.do_mission(first, second)
        self.accel(self.speed)

    def set_mission(self, mission_num):
        self.mission_num = mission_num

    def do_mission(self, first, second):
        """
        TODO: @박준혁 first 와 second 값 설명 주석 써 주기
        :param first: first 값은 무엇인가요?
        :param second: second 값은 무엇인가요?
        :return: 리턴 값은 없습니다.
        """
        if self.mission_num == 0:  # default 주행
            if first is None:
                return
            self.__default__(first[0] / 100, first[1])

        elif self.mission_num == 1:  # 유턴
            self.__turn__(first / 100)

        elif self.mission_num == 2:  # 횡단보도
            if first is None:
                self.__cross__(100)
            else:
                self.__cross__(first / 100)

        elif self.mission_num == 3:  # 간격 유지 주행
            self.__target__(first / 100)

        elif self.mission_num == 4:  # 주차
            # TODO: 후에 추가로 작성하기
            return 0

        else:
            self.__obs__(first[0] / 100, first[1])

    def accel(self, speed):  # TODO: 후에 실험값을 통해서 값 수정
        if self.speed_platform < (speed / 2):
            self.accel_speed = 2 * self.speed
        else:
            self.accel_speed = self.speed

        self.accel_brake = self.brake
        self.accel_steer = self.steer
        self.accel_gear = self.gear

    def write(self):
        return self.accel_gear, self.accel_speed, self.accel_steer, self.accel_brake

    def __default__(self, cross_track_error, linear):
        gear = 0
        brake = 0

        tan_value = linear * (-1)
        theta_1 = math.degrees(math.atan(tan_value))

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
            speed = 60  # TODO: 실험값 수정하기
        else:
            speed = 72  # TODO: 실험값 수정하기

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

    def __turn__(self, turn_distance):
        gear = 0
        speed = 60
        steer = 0
        brake = 0

        self.change_mission = 0

        if self.u_sit == 0:
            if turn_distance < 3.50:
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

            if (self.ct2 - self.ct1) < 530:  # TODO: 실험값 수정
                steer = -1970

            elif 530 <= (self.ct2 - self.ct1) <= 630:  # TODO: 실험값 수정
                speed = 30
                steer = -1970

            elif (self.ct2 - self.ct1) >= 630:
                steer = 0

                if self.speed_platform == 0:
                    self.u_sit = 2

        elif self.u_sit == 2:
            if self.ct3 == 0:
                self.ct3 = self.enc_platform
            self.ct4 = self.enc_platform

            if (self.ct4 - self.ct3) < 75:
                speed = 60
                steer = 1970
                brake = 0

            if 75 <= (self.ct4 - self.ct3) <= 145:
                speed = 30
                steer = 1970

            elif (self.ct4 - self.ct3) >= 145:
                speed = 30
                steer = 0

                if self.speed_platform == 0:
                    self.u_sit = 3

                self.change_mission = 1

        elif self.u_sit == 3:
            if self.ct5 == 0:
                self.ct5 = self.enc_platform
            self.ct6 = self.enc_platform

            if (self.ct6 - self.ct5) < 50:
                speed = 60
                steer = 0
                brake = 0

            if (self.ct6 - self.ct5) >= 50:
                speed = 72
                steer = 0

                if self.speed_platform == 0:
                    self.u_sit = 4

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

    def __obs__(self, obs_r, obs_theta):
        gear = 0
        brake = 0

        obs_mode = 0
        car_circle = 1

        if self.mission_num == 2:
            speed = 60
            correction = 1.6
            adjust = 0.05
            obs_mode = 0

        else:
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

        else:
            if obs_mode == 0:
                if obs_theta == -35:
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

    def __cross__(self, stop_line):
        steer = 0
        speed = 60
        gear = 0
        brake = 0

        self.change_mission = 0

        if abs(stop_line) < 1.7:  # TODO: 기준선까지의 거리값, 경로생성 알고리즘에서 값 받아오기
            if self.t1 == 0:
                self.t1 = time.time()
            self.t2 = time.time()

            if (self.t2 - self.t1) < 4.0:
                speed = 0
                brake = 80
            else:
                speed = 60
                brake = 0
                self.change_mission = 1

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

        time = 0.5  # 값 갱신 속도

        if self.t_sit == 0:
            if distance < 1.5:
                speed = 0
                brake = 80

                self.t_sit = 1

        elif self.t_sit == 1:
            velocity = distance - 1.5

            if velocity < 0:
                velocity = 0

            speed = (velocity * 3.6) / time
            if velocity == 0:
                if distance < 1:
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
