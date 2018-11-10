import pycuda.driver as drv
from pycuda.compiler import SourceModule
import time
import cv2
import numpy as np

import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data


class MotionPlanner(Subroutine):
    def __init__(self, data: Data):
        super().__init__(data)
        # TODO: init할 것 생각하기

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
        # pycuda alloc end

        time.sleep(2)

    def main(self):
        while True:
            self.static_obs_handling(300, 110, 65, 2)
            # 0. Default 주행 상황

            # 1. 협로 주행 상황

            # 2. 유턴 상황

            # 3. 횡단보도 상황

            # 4. 차량추종 상황

            # 5. 주차 상황

        # TODO: main함수 채우기

    # TODO: 미션별로 필요한, main 속에서 loop로 돌릴 메서드 생각하기
    def stop(self):
        # pycuda dealloc
        global context
        context.pop()
        context = None
        from pycuda.tools import clear_context_caches
        clear_context_caches()
        # pycuda dealloc end

    def static_obs_handling(self, radius, angle, obs_size, timeout):
        RAD = np.int32(radius)
        AUX_RANGE = np.int32((180 - angle) / 2)

        lidar_raw_data = self.data.lidar_data_list
        current_frame = np.zeros((RAD, RAD * 2), np.uint8)

        points = np.full((361, 2), -1000, np.int)  # 점 찍을 좌표들을 담을 어레이 (x, y), 멀리 -1000 으로 채워둠.

        for theta in range(0, 361):
            r = lidar_raw_data[theta] / 10  # 차에서 장애물까지의 거리, 단위는 cm

            if 2 <= r:  # 라이다 바로 앞 1cm 의 노이즈는 무시

                # r-theta 를 x-y 로 바꿔서 (실제에서의 위치, 단위는 cm)
                x = -r * np.cos(np.radians(0.5 * theta))
                y = r * np.sin(np.radians(0.5 * theta))

                # 좌표 변환, 화면에서 보이는 좌표(왼쪽 위가 (0, 0))에 맞춰서 집어넣는다
                points[theta][0] = round(x) + RAD
                points[theta][1] = RAD - round(y)

        for point in points:  # 장애물들에 대하여
            cv2.circle(current_frame, tuple(point), obs_size, 255, -1)  # 캔버스에 점 찍기

        data_ = np.zeros((angle + 1, 2), np.int)

        if current_frame is not None:
            self.path(drv.InOut(data_), drv.In(RAD), drv.In(AUX_RANGE), drv.In(current_frame),
                      drv.In(np.int32(RAD * 2)),
                      block=(angle + 1, 1, 1))

        count_ = np.sum(np.transpose(data_)[0])

        if count_ == 0:
            self.lap_during_clear = time.time()

        else:
            self.lap_during_collision = time.time()

        # 다음 세 가지 조건을 모두 만족하면 탈출한다:
        # 전방이 깨끗한 시간이 timeout 이상일 때
        # 장애물을 한 번이라도 만난 뒤에
        # 미션을 시작한 지 3초 이상 지난 뒤에 (표지판을 인식하고 미션을 수행하기 전 탈출하는 것을 방지)
        if self.lap_during_clear - self.lap_during_collision >= timeout and self.lap_during_collision != 0 and \
                time.time() - self.mission_start_lap > 5:
            self.lap_during_clear = 0
            self.lap_during_collision = 0
            self.mission_start_lap = 0
            self.mission_num = 0
            self.data.detected_mission_number = 0

        data = np.zeros((angle + 1, 2), np.int)

        color = None
        target = None

        if current_frame is not None:
            self.path(drv.InOut(data), drv.In(RAD), drv.In(AUX_RANGE), drv.In(current_frame), drv.In(np.int32(RAD * 2)),
                      block=(angle + 1, 1, 1))

            data_transposed = np.transpose(data)

            # 장애물에 부딫힌 곳까지 하얀 선 그리기
            for i in range(0, angle + 1):
                x = RAD + int(data_transposed[1][i] * np.cos(np.radians(i + AUX_RANGE))) - 1
                y = RAD - int(data_transposed[1][i] * np.sin(np.radians(i + AUX_RANGE))) - 1
                cv2.line(current_frame, (RAD, RAD), (x, y), 255)

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

            if np.sum(data_transposed[1]) == 0:
                r = 0
                found = False
                while not found:
                    for theta in (AUX_RANGE, 180 - AUX_RANGE):
                        x = RAD + int(r * np.cos(np.radians(theta))) - 1
                        y = RAD - int(r * np.sin(np.radians(theta))) - 1

                        if current_frame[y][x] == 0:
                            found = True
                            target = -theta
                            break
                    r += 1

            if target >= 0:
                if self.previous_data is not None and abs(
                        self.previous_data[self.previous_target - AUX_RANGE][1] - data[target - AUX_RANGE][1]) <= 1 and \
                        data[target - AUX_RANGE][1] != RAD - 1:
                    target = self.previous_target

                x_target = RAD + int(data_transposed[1][int(target) - AUX_RANGE] * np.cos(np.radians(int(target))))
                y_target = RAD - int(data_transposed[1][int(target) - AUX_RANGE] * np.sin(np.radians(int(target))))
                cv2.line(color, (RAD, RAD), (x_target, y_target), (0, 0, 255), 2)

                self.motion_parameter = (self.mission_num, (data_transposed[1][target - AUX_RANGE], target), None,
                                         None)

                self.previous_data = data
                self.previous_target = target

            else:
                x_target = RAD + int(100 * np.cos(np.radians(int(-target)))) - 1
                y_target = RAD - int(100 * np.sin(np.radians(int(-target)))) - 1
                cv2.line(color, (RAD, RAD), (x_target, y_target), (0, 0, 255), 2)

                self.motion_parameter = (self.mission_num, (10, target), None, None)

            if color is None: return


if __name__ == "__main__":
    import threading
    from lidar import Lidar

    testDT = Data()
    testMP = MotionPlanner(testDT)
    current_lidar = Lidar(testDT)

    testMP_thread = threading.Thread(target=testMP.main)
    lidar_test_thread = threading.Thread(target=current_lidar.main)

    lidar_test_thread.start()
    testMP_thread.start()

    testMP.stop()
