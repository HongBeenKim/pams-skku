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
        #######################################
        self.mission_num = 0  # DEFAULT 모드
        #######################################
        self.steer_past = 0
        self.speed_past = 0
        self.front_distance = 0
        self.theta_turn = 0
        #######################################
        self.u_sit = 0
        self.c_sit = 0
        self.t_sit = 0
        self.p_sit = 0
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
        #######################################
        self.check = [0, 0]

    def main(self):
        while True:
            packet = self.read_packet_from_planner()
            speed_platform, enc_platform = self.read_car_platform_status()
            gear, speed, steer, brake = self.do_mission(packet, speed_platform, enc_platform)
            a_speed, a_brake = self.__accel__(speed, brake)
            self.data.set_control_value(gear, a_speed, steer, a_brake)
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
        return speed, enc

    def do_mission(self, packet, speed_platform, enc_platform):
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
            TODO: 명료화 필요.
            packet[1] =  Lidar에서 180도 값 수치 (벽에서 차량정면에 내린 수선의 발의 길이) (Wiki 참고) 
            packet[2] =  차량 정면의 법선벡터와 x축과의 거리
        
        3. crosswalk
            packet[1] = 정지선으로 부터 거리, 넘겨받는 단위: cm
        4. target_tracking
            packet[1] = target car 와의 거리, 넘겨받는 단위: cm
        5. parking
            TODO: ???
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

        gear = speed = steer = brake = 0
        if self.mission_num == self.data.MODES["default"]:
            if packet is None:
                return 0, 0, 0, 0
            gear, speed, steer, brake = self.__default__(packet[1] / 100, packet[2])

        elif self.mission_num == self.data.MODES["narrow"]:
            gear, speed, steer, brake = self.__obs__(packet[1] / 100, packet[2])

        elif self.mission_num == self.data.MODES["u_turn"]:
            gear, speed, steer, brake = self.__turn__(packet[1] / 100, packet[2], packet[3] / 100)
            # TODO: 수정

        elif self.mission_num == self.data.MODES["crosswalk"]:
            if packet is None:
                gear, speed, steer, brake = 0, 0, 0, 0
            else:
                gear, speed, steer, brake = self.__cross__(packet[1] / 100)

        elif self.mission_num == self.data.MODES["target_tracking"]:
            if packet[1] is None:
                packet[1] = 6
            gear, speed, steer, brake = self.__target__(packet[1] / 100, packet[2], packet[3] / 100)

        elif self.mission_num == self.data.MODES["parking"]:
            gear, speed, steer, brake = self.__parking__(packet[1] / 100, packet[2] / 100, packet[3], packet[4] / 100)

        return gear, speed, steer, brake

    def __accel__(self, speed, brake):  # TODO: 후에 실험값을 통해서 값 수정
        if self.speed_platform < ((3 * speed) / 5):
            if speed <= 10:
                self.accel_speed = 30  # TODO: 후에 실험값을 통해서 값 수정
                self.accel_brake = brake
            else:
                self.accel_speed = 2 * speed
                self.accel_brake = brake
        elif self.speed_platform > (speed / 3):
            self.accel_speed = 0
            self.accel_brake = 20
        else:
            self.accel_speed = speed
            self.accel_brake = brake
        return self.accel_speed, self.accel_brake

    def __default__(self, cross_track_error, theta):
        if self.check[0] != 0:
            self.check[1] = 1
            self.data.check_mission_completed("narrow")

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

        return self.gear, self.speed, self.steer, self.brake

    def __obs__(self, obs_r, obs_theta):
        if self.check[1] == 0:
            self.check[0] = 1

        gear = 0
        brake = 0

        correction = 1.5  # 1.8
        adjust = 0.05
        car_circle = 1

        speed_mission = 36  # TODO: 미션용 속도, 실험하고 변경바람
        speed = speed_mission  # TODO: 연습장 상태가 좋지 않음(기울기 존재)

        if obs_r < 2:
            speed = 24

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
                speed = 30

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

        return self.gear, self.speed, self.steer, self.brake

    def __turn__(self, turn_distance, front_theta, right_distance):  # (전방 장애물 거리, 전방 각도, 오른쪽 직각 거리)
        gear = 0
        speed = 60
        steer = 0
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
            steer = self.__turn_steer__(term)
            self.right_distance_past = self.right_distance

            if 4.0 < self.front_distance < 4.5:
                speed = 35
            elif self.front_distance < 4.0:
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    self.theta_turn = front_theta
                    steer = 0
                    self.u_sit = 1

        elif self.u_sit == 1:
            correction_enc = ((90 - self.theta_turn) / 180) * 786
            speed = 60
            if self.ct1 == 0:
                self.ct1 = self.enc_platform
            self.ct2 = self.enc_platform

            if (self.ct2 - self.ct1) < 630:  # TODO: 실험값 수정
                steer = -1469.79

            elif 530 <= (self.ct2 - self.ct1) <= 760 + correction_enc:  # TODO: 실험값 수정
                speed = 30
                steer = -1469.79

            elif (self.ct2 - self.ct1) >= 760 + correction_enc:
                steer = -1469.79
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
        steer = math.degrees(math.atan((k * term) / velocity))

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
        speed = 60
        gear = 0
        brake = 0

        if self.c_sit == 0:
            if 1.5 < abs(stop_line) < 2:  # TODO: 기준선까지의 거리값, 경로생성 알고리즘에서 값 받아오기
                speed = 35
            elif abs(stop_line) < 1.5:
                speed = 0
                brake = 60
                self.c_sit = 1

        elif self.c_sit == 1:
            if light_signal is True:
                self.data.check_mission_completed("crosswalk")
            else:
                speed = 0
                brake = 0

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __target__(self, distance, cross_track_error, theta):  # (차량과의 거리, cross_track_error, 차선 기울기 각도)
        speed = 50
        gear = 0
        brake = 0

        steer = self.__target_steer__(cross_track_error, theta)

        time_change = 0.05  # 값 갱신 속도, 수정바람

        if self.t_sit == 0:
            if 1.5 < distance < 2.0:
                speed = 30

            elif distance < 1.5:
                speed = 0
                brake = 80
                self.t_sit = 1

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

            if distance is 6:  # TODO: 실험값 수정
                self.data.check_mission_completed("target_tracking")

        self.gear = gear
        self.speed = speed
        self.steer = steer
        self.brake = brake

        return self.gear, self.speed, self.steer, self.brake

    def __target_steer__(self, cross_track_error, theta):
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
        speed = 60
        steer = 0
        brake = 0

        if self.p_sit == 0:
            if 2 < front_distance < 3:
                speed = 40
            elif front_distance < 2:
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    brake = 0
                    self.park_front_distance = front_distance / 2 + 0.90
                    self.park_line_distance = line_distance / 2
                    self.park_line_theta = (90 - line_theta)
                    self.p_sit = 1

        elif self.p_sit == 1:
            if self.pt1 == 0:
                self.pt1 = self.enc_platform
            self.pt2 = self.enc_platform

            term_1 = self.pt1 - self.pt2

            if term_1 > -80:
                gear = 2
                speed = 50

            elif term_1 <= -80:
                speed = 0
                brake = 60
                gear = 0

                if self.speed_platform == 0:
                    self.p_sit = 2

        elif self.p_sit == 2:
            if self.pt3 == 0:
                self.pt3 = self.enc_platform
            self.pt4 = self.enc_platform

            term_2 = self.p4 - self.pt3

            if self.park_line_theta <= 0:
                self.park_line_theta = abs(self.park_line_theta)

                theta = math.atan(self.park_front_distance / abs(self.park_line_distance))
                theta_1 = math.radians(theta)
                theta_2 = (90 - theta)
                theta_handle = math.radians(theta_2)

                radius = self.front_distance / math.cos(theta_1)

                enc_park = radius * theta_handle / 1.697

                steer_handle = (1970 / 19.387) * theta_handle * -1

                if term_2 < enc_park - 15:
                    steer = steer_handle
                    speed = 40

                elif term_2 >= enc_park - 15:
                    steer = steer_handle
                    speed = 0
                    brake = 60

                    if self.speed_platform == 0:
                        steer = 0
                        self.p_sit = 3
            else:
                self.park_line_theta = abs(self.park_line_theta)

                theta = math.atan(self.park_front_distance / abs(self.park_line_distance))
                theta_1 = math.radians(theta)
                theta_2 = (90 - theta)
                theta_handle = math.radians(theta_2)

                radius = self.front_distance / math.cos(theta_1)

                enc_park = (radius * theta_handle / 1.697) * 1.173

                steer_handle = (1970 / 19.387) * theta_handle

                if term_2 < enc_park - 15:
                    steer = steer_handle
                    speed = 40

                elif term_2 >= enc_park - 15:
                    steer = steer_handle
                    speed = 0
                    brake = 60

                    if self.speed_platform == 0:
                        steer = 0
                        self.p_sit = 3

        elif self.p_sit == 3:
            gear = 0
            steer = self.__parking_steer__(line_distance, line_theta)

            if stop_distance > 1.5:
                speed = 40
            elif 1 < stop_distance < 1.5:
                speed = 20
            elif line_distance < 1:
                speed = 0
                brake = 60

                if self.speed_platform == 0:
                    steer = 0
                    speed = 0
                    brake = 0
                    self.p_sit = 4

        elif self.p_sit == 4:
            steer = 0
            speed = 0
            brake = 0

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
            theta_2 = math.degrees(math.atan((k * cross_track_error) / velocity))

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
