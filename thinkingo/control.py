# TODO: 신호등 인식이 끝까지 안될 경우 10초 대기 후 출발 매크로 짜기
# TODO: 횡단보도 신호등 매크로 짜기
# TODO: 주차 매크로 두 가지 버전으로 짜기: 대충 주차공간 인식할 수 있는 각도로 틀어주기 & 주차 완료하기
import sys

sys.path.append(".")

from subroutine import Subroutine
from data_class import Data

import time
import math


class Control(Subroutine):

    def __init__(self, data: Data):
        super().__init__(data)

        self.mode = 0
        self.mode1 = 0
        self.mode2 = 0
        self.st1 = 0
        self.st2 = 0


        self.steer = 0  # 계산된 조향각
        self.speed = 0  # 계산된 속도
        self.gear = 0  # 계산된 기어
        self.brake = 0  # 계산된 브레이크

        self.speed_platform = 0  # 플랫폼에서 읽은 현재 속도
        self.enc_platform = 0  # 플랫폼에서 읽은 엔코더 값

        self.accel_speed = 0  # 가속된 최종 조향각
        self.accel_brake = 0  # 가속된 최종 브레이크
        #######################################
        self.mission_num = 0  # DEFAULT 모드
        self.packet = [0, 0, 0, 0, 0]
        #######################################
        self.steer_past = 0
        self.speed_past = 0
        self.front_distance = 0
        self.theta_turn = 0
        #######################################
        self.d_sit = 0
        self.u_sit = 0
        self.c_sit = 0
        self.t_sit = 0
        self.p_sit = 0
        #######################################
        self.dt1 = 0
        self.dt2 = 0
        #######################################
        self.target_steer = 0
        self.target_steer_past = 0
        #######################################
        self.right_distance = 0
        self.right_distance_past = 0
        self.turn_steer = 0
        self.turn_steer_past = 0
        self.ct1 = 0
        self.ct2 = 0
        #######################################
        self.park_front_distance = 0
        self.park_line_distance = 0
        self.park_line_theta = 0
        self.park_steer = 0
        self.park_steer_past = 0
        self.pt1 = 0
        self.pt2 = 0
        self.pt3 = 0
        self.pt4 = 0
        self.pt5 = 0
        self.pt6 = 0
        #######################################
        self.start = 0
        self.st1 = 0
        self.st2 = 0
        self.pmode = 0
        self.direction = 1

    def main(self):
        while True:
            packet = self.read_packet_from_planner()
            speed_platform, pmode, enc_platform = self.read_car_platform_status()
            gear, speed, steer, brake = self.do_mission(packet, speed_platform, enc_platform, pmode)
            print(self.pmode)
            if self.mode == 0:
                a_speed, a_brake = self.__accel__(speed, brake)
            elif self.mode == 1:
                a_speed, a_brake = speed, brake
            self.data.set_control_value(gear, a_speed, steer, a_brake)
            # self.data.set_control_value(self.__start__(gear, a_speed, steer, a_brake))
            time.sleep(0.01)
            if self.data.is_all_system_stop():
                break

    def read_packet_from_planner(self):
        """
        data_class.py 를 통해 planner 에서 온 데이터 받아오기
        :return: the packet and second packet from planner
        """
        packet = self.data.planner_to_control_packet
        self.mission_num = packet[0]  # set mission number
        return packet

    def read_car_platform_status(self):
        """
        data_class.py 에서 차량 플랫폼 데이터 받아오기
        """
        gear, speed, steer, brake, aorm, alive, enc = self.data.car_platform_status()
        return speed, aorm, enc

    def do_mission(self, packet, speed_platform, enc_platform, pmode):
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
            packet[1] = 최대 부채살의 길이
            packet[2] = 최대 부채살의 각도
        
        2. u_turn
            packet[1] = 전방 거리
            packet[2] =  전방 각도
            packet[3] = 오른쪽 거리 (안씀, 하지만 그냥 내버려둠)
            #TODO: packet[4] 채우기 @김홍빈
            packet[4] = 차선의 바닥 절편만 건네주기, 차선 없으면 None
        
        3. crosswalk
            packet[1] = 정지선으로 부터 거리, 넘겨받는 단위: cm
        4. target_tracking
            packet[1] = target car 와의 거리, 넘겨받는 단위: cm
        5. parking
            TODO: 매크로 만들기 @박준혁
        :return: 리턴 값은 없습니다.
        """

        if speed_platform is None:
            self.speed_platform = 0
        else:
            self.speed_platform = speed_platform
        if enc_platform is None:
            self.enc_platform = 0
        else:
            self.enc_platform = enc_platform

        self.pmode = pmode
        print(self.pmode)

        gear = speed = steer = brake = 0
        if self.mission_num == self.data.MODES["default"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None or packet[2] is None:
                self.packet[1] = 0
                self.packet[2] = 90

            gear, speed, steer, brake = self.__default__(self.packet[1] / 100, self.packet[2])

        elif self.mission_num == self.data.MODES["narrow"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None or packet[2] is None:
                return 0, 0, 0, 0
            gear, speed, steer, brake = self.__obs__(self.packet[1] / 100, self.packet[2])

        elif self.mission_num == self.data.MODES["u_turn"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None:
                self.packet[1] = 600
            if packet[2] is None or packet[3] is None:
                self.packet[2] = 0
                self.packet[3] = 90

            print(self.packet[1])

            gear, speed, steer, brake = self.__turn__(self.packet[1] / 100, self.packet[2], self.packet[3] / 100)

        elif self.mission_num == self.data.MODES["crosswalk"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None:
                gear, speed, steer, brake = 0, 30, 0, 0
            else:
                gear, speed, steer, brake = self.__cross__(self.packet[1] / 100, self.packet[2])

        elif self.mission_num == self.data.MODES["target_tracking"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None:
                self.packet[1] = 600

            if packet[2] is None or packet[3] is None:
                self.packet[2] = 0
                self.packet[3] = 90

            gear, speed, steer, brake = self.__target__(self.packet[1] / 100, self.packet[2] / 100, self.packet[3])

        elif self.mission_num == self.data.MODES["parking"]:
            self.packet[0] = packet[0]
            self.packet[1] = packet[1]
            self.packet[2] = packet[2]
            self.packet[3] = packet[3]
            self.packet[4] = packet[4]

            if packet[1] is None:
                self.packet[1] = 600
            if packet[2] is None:
                self.packet[2] = 0
            if packet[3] is None:
                self.packet[3] = 0
            if packet[4] is None:
                self.packet[4] = 3

            gear, speed, steer, brake = self.__parking__(self.packet[1] / 100, self.packet[2] / 100, self.packet[3], self.packet[4] / 100)

        return gear, speed, steer, brake

    def __accel__(self, speed, brake):
        if self.speed_platform < (speed / 2):
            self.accel_speed = speed * 2
            self.accel_brake = brake
        else:
            self.accel_speed = speed
            self.accel_brake = brake

        return self.accel_speed, self.accel_brake

    def __start__(self, gear, speed, steer, brake):
        if self.start == 0:
            if self.speed_platform > 1:
                self.start = 1

            if self.pmode == 0:
                if self.st1 == 0:
                    self.st1 = time.time()
                self.st2 = time.time()
                if self.st2 - self.st1 > 10:
                    self.start = 1
                    return 0, 30, 0, 0
        else:
            return gear, speed, steer, brake

        if self.start == 1:
            return gear, speed, steer, brake

    def __default__(self, cross_track_error, theta):
        if self.d_sit == 0:

            if self.dt1 == 0:
                self.dt1 = self.enc_platform
            self.dt2 = self.enc_platform

            term = (self.dt2 - self.dt1)

            if term < 50:
                return 0, 24, 0, 0  # 속도 30

            elif term > 50:
                gear = 0
                brake = 0

                theta_1 = 0  # (90 - theta)

                k = 1
                # if abs(theta_1) < 15 and abs(cross_track_error) < 0.27:
                #    k = 0.5

                if self.speed_platform == 0:
                    theta_2 = 0
                else:
                    velocity = (self.speed_platform * 100) / 3600
                    theta_2 = math.degrees(math.atan((k * cross_track_error) / (velocity + 0.01)))

                steer_now = (theta_1 + theta_2)

                adjust = 0.3

                steer_final = ((adjust * self.steer_past) + ((1 - adjust) * steer_now))
                self.steer_past = steer_final

                steer = steer_final * 71
                if steer > 1970:
                    steer = 1970
                    self.steer_past = 27.746
                elif steer < -1970:
                    steer = -1970
                    self.steer_past = -27.746

                if abs(steer_now) > 15:
                    speed = 24
                else:
                    speed = 24

                self.gear = gear
                self.speed = speed
                self.steer = steer
                self.brake = brake

                return self.gear, self.speed, self.steer, self.brake

    def __obs__(self, obs_r, obs_theta):
        gear = 0
        brake = 0

        correction = 1.5  # 1.8
        adjust = 0.05
        car_circle = 1

        speed_mission = 36
        speed = speed_mission

        if obs_r < 2:
            speed = 20

        if self.speed_platform > speed_mission:  # 기울기가 있어서 가속받을 경우 급정지
            speed = 0
            brake = 60

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
                speed = 20

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

        if self.speed_platform > speed:  # 기울기가 있어서 가속받을 경우 급정지
            speed = 0
            brake = 60

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __turn__(self, turn_distance, front_theta, right_distance):  # (전방 장애물 거리, 전방 각도, 오른쪽 직각 거리)
        gear = 0
        speed = 30   # 60
        steer = - 0.5
        brake = 0

        radian_theta = math.radians(abs(90 - front_theta))
        self.front_distance = turn_distance * math.cos(radian_theta)
        self.right_distance = right_distance

        if self.right_distance_past == 0:
            self.right_distance_past = self.right_distance

        if abs(self.right_distance - self.right_distance_past) > 1:
            self.right_distance = self.right_distance_past

        term = (self.right_distance - self.right_distance_past)

        if self.u_sit == 0:
            steer = 0  # self.__turn_steer__(term * 1)
            self.right_distance_past = self.right_distance

            if self.front_distance < 6:
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    self.theta_turn = front_theta
                    steer = 0
                    self.u_sit = 1
            else:
                speed = 30

        elif self.u_sit == 1:
            correction_enc = ((90 - self.theta_turn) / 180) * 786
            speed = 30  # 60
            if self.ct1 == 0:
                self.ct1 = self.enc_platform
            self.ct2 = self.enc_platform

            if (self.ct2 - self.ct1) < 600:
                steer = -1400.79

            elif 530 <= (self.ct2 - self.ct1) <= 750 + correction_enc:  # TODO: 엔코더 수정하기 @박준혁
                speed = 24  # 30
                steer = -1400.79

            elif (self.ct2 - self.ct1) >= 750 + correction_enc:
                steer = -1400.79
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    steer = 0
                    speed = 0
                    self.data.check_mission_completed("u_turn")

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __turn_steer__(self, term):
        k = 1
        if abs(term) < 0.27:
            k = 0.5

        adjust = 0.5

        velocity = (self.speed_platform * 100) / 3600
        steer = math.degrees(math.atan((k * term) / (velocity + 0.01)))

        steer_final = ((adjust * self.turn_steer_past) + ((1 - adjust) * steer))

        self.turn_steer_past = steer_final

        self.steer = steer_final * 71
        if steer > 1970:
            self.steer = 1970
            self.turn_steer_past = 27.746
        elif steer < -1970:
            self.steer = -1970
            self.turn_steer_past = -27.746

        return self.steer

    def __cross__(self, stop_line, light_signal):  # (정지선 거리, 신호등 신호)
        steer = 0
        speed = 30  # 60
        gear = 0
        brake = 0

        if self.c_sit == 0:
            if 1.5 < abs(stop_line) < 2:
                speed = 24  # 35
            elif abs(stop_line) < 1.5:
                speed = 0
                brake = 60
                self.c_sit = 1
            else:
                speed = 30

        elif self.c_sit == 1:
            if self.mode1 == 0:
                if light_signal is True:
                    self.data.check_mission_completed("crosswalk")
                else:
                    speed = 0
                    brake = 0
            if self.mode1 == 1:
                if self.ct1 == 0:
                    self.ct1 = time.time()
                self.ct2 = time.time()

                if self.ct2 - self.ct1 < 10:
                    return 0, 0, 0, 0
                elif self.ct2 - self.ct1 > 10:
                    speed = 30
                    brake = 0

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __target__(self, distance, cross_track_error, theta):  # (차량과의 거리, cross_track_error, 차선 기울기 각도)
        speed = 30  # 50
        gear = 0
        brake = 0

        steer = self.__target_steer__(cross_track_error * 1, theta * 1)

        time_change = 0.05  # 값 갱신 속도, 수정바람

        if self.t_sit == 0:
            if distance < 3.2:
                speed = 0
                brake = 60  # 80
                self.t_sit = 1

            else:
                speed = 30

        elif self.t_sit == 1:
            self.speed_past = self.speed_platform
            velocity = distance - 3.0
            speed = self.speed_past + (velocity * 3.6) / time_change

            if velocity < 0:
                if distance < 1:
                    speed = 0
                    brake = 60

            if speed > 35:  # 60
                speed = 35  # 60

            if distance is 6.00:
                self.data.check_mission_completed("target_tracking")
                self.t_sit = 2

        elif self.t_sit == 2:
            return 0, 0, 0, 0

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __target_steer__(self, cross_track_error, theta):
        theta_1 = 0  # (90 - theta)

        k = 1
        #if abs(theta_1) < 15 and abs(cross_track_error) < 0.27:
        #    k = 0.5

        if self.speed_platform == 0:
            theta_2 = 0
        else:
            velocity = (self.speed_platform * 100) / 3600
            theta_2 = math.degrees(math.atan((k * cross_track_error) / (velocity + 0.01)))

        steer_now = (theta_1 + theta_2)

        adjust = 0.3

        steer_final = ((adjust * self.target_steer_past) + ((1 - adjust) * steer_now))
        self.target_steer_past = steer_final

        steer = steer_final * 71
        if steer > 1970:
            steer = 1970
            self.target_steer_past = 27.746
        elif steer < -1970:
            steer = -1970
            self.target_steer_past = -27.746

        self.target_steer = steer
        return self.target_steer

    def __parking__(self, front_distance, line_distance, line_theta, stop_distance):  # (전방 장애물 거리,
        # 주차 공간 선 절편 거리, 주차 공간 양 선의 중심의 기울기 각도, 정지선 거리)
        gear = 0
        speed = 30  # 60
        steer = 0
        brake = 0

        if self.p_sit == 0:
            if front_distance < 3.2:
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    brake = 0
                    self.p_sit = 1
            else:
                speed = 30

        elif self.p_sit == 1:
            if self.direction == 0:

                if self.pt1 == 0:
                    self.pt1 = self.enc_platform
                self.pt2 = self.enc_platform

                term_1 = self.pt2 - self.pt1

                if term_1 < 135:
                    steer = -1970
                    speed = 20  # 50
                elif term_1 > 135:
                    steer = -1970
                    speed = 0
                    brake = 80

                    if self.speed_platform == 0:
                        self.p_sit = 2

            elif self.direction == 1:
                if self.pt1 == 0:
                    self.pt1 = self.enc_platform
                self.pt2 = self.enc_platform

                term_1 = self.pt2 - self.pt1

                if term_1 < 163:
                    steer = 1970
                    speed = 20  # 50
                elif term_1 > 163:
                    steer = 1970
                    speed = 0
                    brake = 80

                    if self.speed_platform == 0:
                        self.p_sit = 2

        elif self.p_sit == 2:
            if self.mode2 == 0:
                if self.pt3 == 0:
                    self.pt3 = self.enc_platform
                self.pt4 = self.enc_platform

                term_2 = self.pt4 - self.pt3

                if self.direction == 0:

                    if term_2 < 163:
                        steer = 1970
                        speed = 20  # 50
                    elif term_2 > 163:
                        steer = 1970
                        speed = 0
                        brake = 80

                        if self.speed_platform == 0:
                            self.p_sit = 3

                if self.direction == 1:
                    if term_2 < 135:
                        steer = -1970
                        speed = 20  # 50
                    elif term_2 > 135:
                        steer = -1970
                        speed = 0
                        brake = 80

                        if self.speed_platform == 0:
                            self.p_sit = 3
            elif self.mode2 == 1:
                self.p_sit = 4
                return 0, 0, 0, 0

        elif self.p_sit == 3:
            if self.pt5 == 0:
                self.pt5 = self.enc_platform
            self.pt6 = self.enc_platform

            term_3 = self.pt6 - self.pt5

            if self.direction == 0:

                if term_3 < 180:
                    steer = 0
                    speed = 20  # 50
                elif term_3 > 180:
                    steer = 0
                    speed = 0
                    brake = 80

                    if self.speed_platform == 0:
                        self.p_sit = 5

            elif self.direction == 1:
                if term_3 < 180:
                    steer = 0
                    speed = 20  # 50
                elif term_3 > 180:
                    steer = 0
                    speed = 0
                    brake = 80

                    if self.speed_platform == 0:
                        self.p_sit = 5


        elif self.p_sit == 4:
            steer = self.__parking_steer__(line_distance, line_theta)

            if stop_distance < 1.4:
                speed = 20
            elif stop_distance < 1.4:
                speed = 0
                brake = 60
                if self.speed_platform == 0:
                    self.p_sit = 5

        elif self.p_sit == 5:
            return 0, 0, 0, 0

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __parking_steer__(self, cross_track_error, theta):
        theta_1 = (90 - theta)

        k = 1
        if abs(theta_1) < 15 and abs(cross_track_error) < 0.27:
            k = 0.5

        if self.speed_platform == 0:
            theta_2 = 0
        else:
            velocity = (self.speed_platform * 100) / 3600
            theta_2 = math.degrees(math.atan((k * cross_track_error) / velocity + 0.01))

        steer_now = (theta_1 + theta_2)

        adjust = 0.3

        steer_final = ((adjust * self.park_steer_past) + ((1 - adjust) * steer_now))
        self.park_steer_past = steer_final

        steer = steer_final * 71
        if steer > 1970:
            steer = 1970
            self.park_steer_past = 27.746
        elif steer < -1970:
            steer = -1970
            self.park_steer_past = -27.746

        self.park_steer = steer
        return self.park_steer


# 테스트용 코드, 아래에 원하는 대로 바꿔서 테스트해볼 수 있습니다.
if __name__ == '__main__':
    import threading

    test_data = Data()
    test_control = Control(test_data)
    control_thread = threading.Thread(target=test_control.main)

    control_thread.start()
