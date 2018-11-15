import pycuda.driver as drv
from pycuda.compiler import SourceModule
import time
import cv2
import numpy as np
import sys

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


#  i.e) U_TURN_ANGLE=30 이면 75도~105도를 읽는다

class MotionPlanner(Subroutine):
    def __init__(self, data_stream: Source, data: Data):
        super().__init__(data)
        # TODO: init 할 것 생각하기
        self.previous_data = None
        self.data_stream = data_stream
        self.lane_handler = LaneCam(data_stream, data)

    def main(self):
        self.init_cuda()  # thread 안에서 initialization 을 해야 합니다.
        while True:
            print("PLANNER current mode: ", self.data.current_mode)
            if self.data_stream.lidar_data is None: continue

            # TODO: sign cam 이랑 미션 들어가있는 여부 공유하는 방법 어떻게 할지 정하기

            # 0. default는 표지판과 차선만 본다
            if self.data.current_mode == self.data.MODES["default"]:
                # TODO: lane_handler에서 값 받아서 패킷에 넘겨주기
                self.data.current_mode = self.data.detected_mission_number

            # 1. 부채살
            elif self.data.current_mode == self.data.MODES["narrow"]:
                # TODO: 직진 매크로를 어디서 구현할지 결정하기
                dist, angle = self.obs_handling(ARC_ANGLE, OBSTACLE_OFFSET)
                self.data.planner_to_control_packet = (self.data.MODES["narrow"], dist, angle, None)

            # 2. 유턴 상황
            elif self.data.current_mode == self.data.MODES["u_turn"]:
                dist, angle = self.U_turn_data(U_TURN_ANGLE)
                self.data.planner_to_control_packet = (self.data.MODES["u_turn"], dist, angle, None)

            # 3. 횡단보도 상황
            elif self.data.current_mode == self.data.MODES["crosswalk"]:
                dist = self.lane_handler.stop_line_detection()
                signal = self.data.light_signal
                self.data.planner_to_control_packet = (self.data.MODES["crosswalk"], dist, signal, None)

            # 4. 차량추종 상황
            elif self.data.current_mode == self.data.MODES["target_tracking"]:
                min_dist = self.calculate_distance_phase_target()
                self.data.planner_to_control_packet = (self.data.MODES["target_tracking"], min_dist, None, None)

            # TODO: 5. 주차 상황
            elif self.data.current_mode == self.data.MODES["parking"]:
                # 주차 미션번호, A or B, 편차, 각도
                # TODO: 라이다로 잰 배리어까지의 거리는 언제 주지?
                # TODO: 패킷 사이즈를 늘린다 vs control 에서 신호를 받아서 주는 패킷 종류를 바꾼다
                pass

            if self.data.is_all_system_stop():
                self.pycuda_deallocation()
                break

        # TODO: main함수 마저 채우기

    # TODO: 미션별로 필요한, main 속에서 loop로 돌릴 메서드 생각하기
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

        # Lidar_data의 자료를 받아온다
        lidar_raw_data = self.data_stream.lidar_data

        # Lidar의 우측 (0도) 까지의 거리를 측정한다.
        lidar_right_distance_mm = lidar_raw_data[0]
        lidar_right_distance_cm = lidar_right_distance_mm / 10  # mm -> cm

        # 전방 시야각을 설정한다. 이 각도의 데이터만 passing할 예정.
        angle_start = 180 - U_angle
        angle_end = 180 + U_angle

        # 읽지 않는 부분을 이 값으로 초기화한다. 이후에 min 함수를 사용하므로 크게 잡았다.
        # lidar의 최대 인식 거리는 20m이므로, 20m = 2000 cm = 20000 mm 임을 감안하여 20001을 대입했다.

        init_list = 20001  # 초기화 숫자. 필요에 따라 음수, 0 혹은 큰 숫자 대입
        lidar_passed_data = []

        for theta in range(0, 360):
            lidar_passed_data[theta] = init_list  # 초기화

        for theta in range(angle_start, angle_end):
            lidar_passed_data[theta] = lidar_raw_data[theta]  # passing

        # 거리 최솟값의 index를 주는 함수
        distance_front_index = lidar_passed_data.index(min(lidar_passed_data))

        # 실제 각도는 index의 1/2
        degree = distance_front_index / 2

        return lidar_right_distance_cm, degree

    def calculate_distance_phase_target(self):
        lidar_raw_data = self.data_stream.lidar_data
        minimum_distance = 10000
        for theta in range(170, 190):
            minimum_distance = min(lidar_raw_data[theta]) / 10  # 전방 좌우 10도의 라이다 값 중 최솟값

        """
        TODO: 과연 최솟값으로 하면 문제가 없을까? 
        튀는 값을 대비하여 3번째 최소인 값을 대입해야 하는 것 아닌가?
        """

        return minimum_distance


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
