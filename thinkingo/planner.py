import pycuda.driver as drv
from pycuda.compiler import SourceModule
import time
import cv2
import numpy as np
import sys
import math

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data
from data_source import Source
from lane_cam import LaneCam

ACTUAL_RADIUS = 500  # 부채살의 실제 반경
CLEAR_RADIUS = 300  # 전방 항시 검사 반경 (부채살과 차선 모드를 넘나들기 위함)
ARC_ANGLE = 110  # 부채살 적용 각도
OBSTACLE_OFFSET = 65  # 부채살 적용 시 장애물의 offset (cm 단위)
U_TURN_ANGLE = 30  # 유턴시 전방 scan 각도. 기준은 90도를 기준으로 좌우 대칭.
U_TURN_LIDAR_CIRCLE_SIZE = 6
U_TURN_LIDAR_LINE_SIZE = 6
RED = (0, 0, 255)
BLUE = (255, 0, 0)


#  i.e) U_TURN_ANGLE=30 이면 75도~105도를 읽는다

class MotionPlanner(Subroutine):
    def __init__(self, data_stream: Source, data: Data):
        super().__init__(data)
        self.previous_data = None
        self.data_stream = data_stream
        self.lane_handler = LaneCam(data_stream)

    def main(self):
        self.init_cuda()  # thread 안에서 initialization 을 해야 합니다.
        while True:
            if self.data_stream.lidar_data is None: continue

            # TODO: sign cam 이랑 미션 들어가있는 여부 공유하는 방법 어떻게 할지 정하기
            time.sleep(0.01)  # for threading schedule
            # 0. default는 표지판과 차선만 본다
            if self.data.current_mode == self.data.MODES["default"]:
                # TODO: lane_handler에서 값 받아서 패킷에 넘겨주기
                frame, intercept, angle = self.lane_handler.lane_detection()
                self.data.planner_to_control_packet = (self.data.MODES["default"], intercept, angle, None, None)
                self.data.planner_monitoring_frame = (frame, 800, 158)
                self.data.current_mode = self.data.detected_mission_number

            # 1. 부채살
            elif self.data.current_mode == self.data.MODES["narrow"]:
                # TODO: 직진 매크로를 어디서 구현할지 결정하기
                if self.is_forward_clear():
                    self.data.current_mode = 0
                    continue
                dist, angle = self.obs_handling(ARC_ANGLE, OBSTACLE_OFFSET)
                self.data.planner_to_control_packet = (self.data.MODES["narrow"], dist, angle, None, None)

            # 2. 유턴 상황
            elif self.data.current_mode == self.data.MODES["u_turn"]:
                front_dist, angle, right_dist = self.U_turn_data(U_TURN_ANGLE)
                self.data.planner_to_control_packet = (self.data.MODES["u_turn"], front_dist, angle, right_dist, None)

            # 3. 횡단보도 상황
            elif self.data.current_mode == self.data.MODES["crosswalk"]:
                frame, dist = self.lane_handler.stop_line_detection()
                signal = self.data.light_signal
                self.data.planner_to_control_packet = (self.data.MODES["crosswalk"], dist, signal, None, None)
                self.data.planner_monitoring_frame = (frame, 600, 300)

            # 4. 차량추종 상황
            elif self.data.current_mode == self.data.MODES["target_tracking"]:
                min_dist, dist_frame = self.calculate_distance_phase_target()
                frame, intercept, angle = self.lane_handler.lane_detection()
                # FIXME: 크기 안 맞음
                self.data.planner_to_control_packet = (self.data.MODES["target_tracking"], min_dist, intercept, angle, None)
                frame = np.concatenate((frame, dist_frame), axis=0)
                self.data.planner_monitoring_frame = (frame, 600, 500)

            # TODO: 5. 주차 상황
            elif self.data.current_mode == self.data.MODES["parking"]:
                frame, dist_to_barrier, interception, angle, stop_dist = self.lane_handler.parking_line_detection()
                self.data.planner_to_control_packet = (self.data.MODES["parking"], dist_to_barrier, interception, angle, stop_dist)
                self.data.planner_monitoring_frame = (frame, 600, 300)

            if self.data.is_all_system_stop():
                self.pycuda_deallocation()
                break

    def pycuda_deallocation(self):
        # pycuda dealloc
        global context
        context.pop()
        context = None
        from pycuda.tools import clear_context_caches
        clear_context_caches()
        # pycuda dealloc end

    def is_forward_clear(self):
        lidar_raw_data = self.data_stream.lidar_data
        for r in lidar_raw_data:
            if 20 <= r < 3000:
                return False
        return True

    def init_cuda(self):
        # pycuda alloc
        drv.init()
        global context
        from pycuda.tools import make_default_context
        context = make_default_context()

        mod = SourceModule(r"""
                        #include <stdio.h>
                        #include <math.h>
                        #define PI 3.14159265
                        __global__ void detect(int data[][2], int* rad, int* range, unsigned char *frame, int *pcol) {
                                for(int r = 0; r < rad[0]; r++) {
                                    const int thetaIdx = threadIdx.x;
                                    const int theta = thetaIdx + range[0];
                                    int x = rad[0] + int(r * cos(theta * PI/180)) - 1;
                                    int y = rad[0] - int(r * sin(theta * PI/180)) - 1;
                                    if (data[thetaIdx][0] == 0) data[thetaIdx][1] = r;
                                    if (*(frame + y * *pcol + x) != 0) data[thetaIdx][0] = 1;
                                }
                        }
                        """)

        self.path = mod.get_function("detect")
        print("PLANNER: pycuda alloc end")
        # pycuda alloc end

        time.sleep(2)

    def obs_handling(self, angle, obs_offset):
        ACT_RAD = np.int32(ACTUAL_RADIUS)  # 실제 라이다 화면의 세로 길이 (즉 부채살의 실제 반경)

        AUX_RANGE = np.int32((180 - angle) / 2)  # 좌우대칭 부채살의 사잇각이 angle, AUX_RANGE 는 +x축 기준 첫 부채살의 각도

        lidar_raw_data = self.data_stream.lidar_data
        current_frame = np.zeros((ACT_RAD, ACT_RAD * 2), np.uint8)  # 그림 그릴 도화지 생성

        points = np.full((361, 2), -1000, np.int)  # 점 찍을 좌표들을 담을 어레이 (x, y), 멀리 -1000 으로 채워둠.

        for theta in range(0, 361):
            r = lidar_raw_data[theta] / 10  # 차에서 장애물까지의 거리, 단위는 cm

            if 2 <= r:  # 라이다 바로 앞 1cm 의 노이즈는 무시

                # r-theta 를 x-y 로 바꿔서 (실제에서의 위치, 단위는 cm)
                x = r * np.cos(np.radians(0.5 * theta))
                y = r * np.sin(np.radians(0.5 * theta))

                # 좌표 변환, 화면에서 보이는 좌표(왼쪽 위가 (0, 0))에 맞춰서 집어넣는다
                points[theta][0] = round(x) + ACT_RAD
                points[theta][1] = ACT_RAD - round(y)

        for point in points:  # 장애물들에 대하여
            cv2.circle(current_frame, tuple(point), obs_offset, 255, -1)  # 캔버스에 점 찍기

        # 부채살의 결과가 저장되는 변수
        data = np.zeros((angle + 1, 2), np.int)

        target = None

        if current_frame is not None:
            # 부채살 호출
            self.path(drv.InOut(data), drv.In(ACT_RAD), drv.In(AUX_RANGE), drv.In(current_frame),
                      drv.In(np.int32(ACT_RAD * 2)), block=(angle + 1, 1, 1))

            # 부채살이 호출되고 나면 data에 부채살 결과가 들어있음
            # (data[theta][0]: theta에서 뭔가에 부딪혔는가?(0 or 1), data[theta][1]: theta에서 뻗어나간 길이)
            data_transposed = np.transpose(data)

            # 장애물에 부딫힌 곳까지 하얀 선 그리기
            for i in range(0, angle + 1):
                x = ACT_RAD + int(data_transposed[1][i] * np.cos(np.radians(i + AUX_RANGE))) - 1
                y = ACT_RAD - int(data_transposed[1][i] * np.sin(np.radians(i + AUX_RANGE))) - 1
                cv2.line(current_frame, (ACT_RAD, ACT_RAD), (x, y), 255)

            # 진행할 방향을 빨간색으로 표시하기 위해 흑백에서 BGR 로 변환
            color = cv2.cvtColor(current_frame, cv2.COLOR_GRAY2BGR)

            # count 는 장애물이 부딪힌 방향의 갯수를 의미
            count = np.sum(data_transposed[0])

            if count <= angle - 1:
                relative_position = np.argwhere(data_transposed[0] == 0) - 90 + AUX_RANGE
                minimum_distance = int(min(abs(relative_position)))

                for i in range(0, len(relative_position)):
                    if abs(relative_position[i]) == minimum_distance:
                        target = int(90 + relative_position[i])

            else:
                target = int(np.argmax(data_transposed[1]) + AUX_RANGE)

            # 차량 바로 앞이 완전히 막혀버렸을 때: 좌로 최대조향할지, 우로 최대조향할지 결정
            # 좌, 우로 부채살 1개씩 뻗어서 먼저 뚫리는 곳으로 최대 조향함
            if np.sum(data_transposed[1]) == 0:
                r = 0
                found = False
                while not found:
                    for theta in (AUX_RANGE, 180 - AUX_RANGE):
                        x = ACT_RAD + int(r * np.cos(np.radians(theta))) - 1
                        y = ACT_RAD - int(r * np.sin(np.radians(theta))) - 1

                        if current_frame[y][x] == 0:
                            found = True
                            target = -theta
                            break
                    r += 1

            if target >= 0:
                # 이건 부채살 길이가 비슷할 때 진동하는 걸 방지하기 위한 필터인데.. 없어도 될듯함
                if self.previous_data is not None and abs(
                        self.previous_data[self.previous_target - AUX_RANGE][1] - data[target - AUX_RANGE][1]) <= 1 and \
                        data[target - AUX_RANGE][1] != ACT_RAD - 1:
                    target = self.previous_target

                x_target = ACT_RAD + int(data_transposed[1][int(target) - AUX_RANGE] * np.cos(np.radians(int(target))))
                y_target = ACT_RAD - int(data_transposed[1][int(target) - AUX_RANGE] * np.sin(np.radians(int(target))))
                cv2.line(color, (ACT_RAD, ACT_RAD), (x_target, y_target), (0, 0, 255), 2)
                max_dist = data_transposed[1][target - AUX_RANGE]

                self.previous_data = data
                self.previous_target = target

            else:
                x_target = ACT_RAD + int(100 * np.cos(np.radians(int(-target)))) - 1
                y_target = ACT_RAD - int(100 * np.sin(np.radians(int(-target)))) - 1
                cv2.line(color, (ACT_RAD, ACT_RAD), (x_target, y_target), (0, 0, 255), 2)
                max_dist = 10

            self.data.planner_monitoring_frame = (color, ACT_RAD * 2, ACT_RAD)

            return max_dist, target

    def U_turn_data(self, U_angle):

        """
        특정 각도에서만 부채살을 적용시키는 프로그램.
        필요에 따라 부채살이 필요할 경우를 대비하여 추가 코드 작업을 할 예정이다
        (2018-11-15 오전 7시까지)

        Input       : U_angle             유턴시 전방 scan 각도. 기준은 90도를 기준으로 좌우 대칭.
                                          i.e) U_TURN_ANGLE=30 이면 75도~105도를 읽는다
    
        Edited 2018-11-15 AM 01:15
        """

        # 전방 시야각을 설정한다. 이 각도의 데이터만 passing할 예정.
        angle_start = 180 - U_angle
        angle_end = 180 + U_angle

        #픽셀 사이즈를 설정한다.
        y_pixel_size = 1000
        x_pixel_size = 2000

        # Lidar_data의 자료를 받아온다
        lidar_raw_data = self.data_stream.lidar_data
        lidar_mat = self.data_stream.get_lidar_ndarray_data(y_pixel_size, x_pixel_size, 10)

        # Lidar의 우측에 선긋기
        lidar_right_distance_mm = lidar_raw_data[0]
        lidar_right_distance_cm = int(lidar_right_distance_mm / 10)  # mm -> cm
        cv2.line(lidar_mat, (int(x_pixel_size / 2), int(y_pixel_size)),
                 (int(x_pixel_size / 2) + lidar_right_distance_cm, int(y_pixel_size)), RED, U_TURN_LIDAR_LINE_SIZE)

        # 전방 최소점에 선긋기
        distance_front_index = np.argmin(np.array(lidar_raw_data)[angle_start:angle_end]) + angle_start
        front_min_dist = lidar_raw_data[distance_front_index] / 10
        front_degree = distance_front_index / 2  # 실제 각도는 index의 1/2
        front_x = int(x_pixel_size / 2) + int(front_min_dist * math.cos(np.radians(front_degree)))
        front_y = int(y_pixel_size) - int(front_min_dist * math.sin(np.radians(front_degree)))
        cv2.line(lidar_mat, (int(x_pixel_size / 2), int(y_pixel_size)), (front_x, front_y), RED, U_TURN_LIDAR_LINE_SIZE)

        # 라이다 위치에 파란점
        cv2.circle(lidar_mat, (int(x_pixel_size / 2), int(y_pixel_size)), 1, BLUE, U_TURN_LIDAR_CIRCLE_SIZE)

        #  이미지 띄우는 곳
        resized = cv2.resize(lidar_mat, (1000, 500))
        self.data.planner_monitoring_frame = (resized, 1000, 500)

        return front_min_dist, front_degree, lidar_right_distance_cm

    def calculate_distance_phase_target(self):
        lidar_raw_data = self.data_stream.lidar_data
        minimum_distance = lidar_raw_data[180] / 10
        min_theta = 0
        car_width = 160 #cm
        
        for theta in range(360):
            if(minimum_distance > lidar_raw_data[theta] / 10) and \
                    ((lidar_raw_data[theta] / 10) * abs(math.cos(theta * 90 / math.pi)) < (car_width / 2)): #2 Conditions
                minimum_distance = lidar_raw_data[theta] / 10
                min_theta = theta
        distance = minimum_distance * math.sin(min_theta * 90 / math.pi)

        """
            [INFO]
            Horizontal line:  A, B     Diagonal line: C     Vertical line : D
            
            A: C * cos(theta)
            B: Car Width / 2
            C: Raw Data
            D: C * sin(theta)
            
            [Condition 1]
            A should be smaller than B
            
            [Condition 2]
            C is minimum distance of lidar raw data which is satisfied
            
                     A - - > X
                            /|
                           / |
                     C    /  | D
                         /   |
                        /    |
                     B - - - - - >
           ------------------------
           |                      |
           |       OUR CAR        |
           |                      |
           
        """
        if distance > 300:  # 300cm 앞까지 물체가 없으면 차가 없어진걸로.. 수치적으로 검토바람
            return
        else:
            img = self.data_stream.get_lidar_ndarray_data(1000, 500, 5)
            distance = int(distance)
            img = cv2.line(img, (distance, 0), (distance, int(500 + distance * math.cos(min_theta * 90 / math.pi)) ), (255, 255, 0), 2)
            img = cv2.putText(img, "%d"%distance, (int(distance / 2) , int(500 + distance * math.cos(min_theta * 90 / math.pi)) ), cv2.FONT_HERSHEY_SIMPLEX, 4, (255,255,0))
            img = cv2.resize(img, (400, 200))
            return distance, img


if __name__ == "__main__":
    import threading
    from control import Control
    from car_platform import CarPlatform
    from monitoring import Monitoring

    testDT = Data()
    """
    test code
    특정 미션 번호에서 시작하도록 함
    """
    testDT.current_mode = 1

    testDS = Source(testDT)
    car = CarPlatform('COM5', testDT)
    testMP = MotionPlanner(testDS, testDT)
    test_control = Control(testDT)
    monitor = Monitoring(testDS, testDT)

    lidar_source_thread = threading.Thread(target=testDS.lidar_stream_main)
    left_cam_source_thread = threading.Thread(target=testDS.left_cam_stream_main)
    right_cam_source_thread = threading.Thread(target=testDS.right_cam_stream_main)
    mid_cam_source_thread = threading.Thread(target=testDS.mid_cam_stream_main)
    planner_thread = threading.Thread(target=testMP.main)
    control_thread = threading.Thread(target=test_control.main)
    car_thread = threading.Thread(target=car.main)
    monitoring_thread = threading.Thread(target=monitor.main)

    lidar_source_thread.start()
    planner_thread.start()
    time.sleep(3)
    car_thread.start()
    control_thread.start()

    left_cam_source_thread.start()
    right_cam_source_thread.start()
    mid_cam_source_thread.start()
    monitoring_thread.start()
