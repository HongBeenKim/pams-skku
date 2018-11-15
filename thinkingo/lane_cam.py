import cv2
import numpy as np
import sys

sys.path.append(".")
sys.path.append("../test")
from data_source import Source
from data_class import Data
import random
import time


class LaneCam():
    camera_matrix_L = np.array([[474.383699, 0, 404.369647], [0, 478.128447, 212.932297], [0, 0, 1]])
    camera_matrix_R = np.array([[473.334870, 0, 386.312394], [0, 476.881433, 201.662339], [0, 0, 1]])
    distortion_coefficients_L = np.array([0.164159, -0.193892, -0.002730, -0.001859])
    distortion_coefficients_R = np.array([0.116554, -0.155379, -0.001045, -0.001512])

    pts1_L = np.float32([[0, 0], [0, 428], [780, 0], [780, 428]])
    pts2_L = np.float32([[0, 258], [403, 427], [526, 0], [523, 347]])

    pts1_R = np.float32([[0, 0], [0, 428], [780, 0], [780, 428]])
    pts2_R = np.float32([[0, 0], [15, 327], [459, 294], [130, 415]])

    Bird_view_matrix_L = cv2.getPerspectiveTransform(pts1_L, pts2_L)
    Bird_view_matrix_R = cv2.getPerspectiveTransform(pts1_R, pts2_R)

    lower_white = np.array([187, 180, 160], dtype=np.uint8)
    upper_white = np.array([255, 254, 255], dtype=np.uint8)

    def __init__(self, data_source: Source):
        self.data_source = data_source

    def make_merged_frame(self):
        if self.data_source.left_frame is None or self.data_source.right_frame is None:
            return None
        left_frame = self.data_source.left_frame.copy()
        right_frame = self.data_source.right_frame.copy()
        undistorted_left = cv2.undistort(left_frame, self.camera_matrix_L,
                                         self.distortion_coefficients_L, None, None)[10:438, 10:790]
        undistorted_right = cv2.undistort(right_frame, self.camera_matrix_R,
                                          self.distortion_coefficients_R, None, None)[10:438, 10:790]

        transformed_left = cv2.warpPerspective(undistorted_left, self.Bird_view_matrix_L, (526, 427))[95:395, 224:524]
        transformed_right = cv2.warpPerspective(undistorted_right, self.Bird_view_matrix_R, (459, 415))[73:373, 15:315]

        merged_frame = np.hstack((transformed_left, transformed_right))
        return merged_frame

    def parking_line_detection(self):
        lidar_raw_data = self.data_source.lidar_data
        front_first = 75 * 2
        front_second = 105 * 2

        # TODO: need to test
        min_dist = min(lidar_raw_data[front_first:front_second])

        merged_frame = self.make_merged_frame()
        filtered_frame = cv2.inRange(merged_frame, self.lower_white, self.upper_white)

        lines = cv2.HoughLinesP(filtered_frame, 1, np.pi / 360, 40, 100, 40)

        t1 = time.time()

        interception = None
        angle = None

        while True:
            if lines is None: break

            if (time.time() - t1) >= 0.01:
                break

            if len(lines) == 1: break
            magnitude_a, magnitude_b, vec_a, vec_b = 0, 0, (0, 0), (0, 0)
            a, b, c, mid_x_b, mid_y_b = 0, 0, 0, 0, 0
            rand = random.sample(list(range(0, len(lines))), 2)

            for x1, y1, x2, y2 in lines[rand[0]]:
                vec_a = (x2 - x1, y1 - y2)
                magnitude_a = np.sqrt(vec_a[0] ** 2 + vec_a[1] ** 2)
                a, b, c = y1 - y2, x1 - x2, (300 - y1) * (x2 - x1) - x1 * (y1 - y2)

            if np.abs(vec_a[1] / vec_a[0]) < 1: continue

            for x1, y1, x2, y2 in lines[rand[1]]:
                vec_b = (x2 - x1, y1 - y2)
                magnitude_b = np.sqrt(vec_b[0] ** 2 + vec_b[1] ** 2)
                mid_x_b, mid_y_b = (x1 + x2) / 2, 300 - (y1 + y2) / 2

            cos_theta = (vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1]) / (magnitude_a * magnitude_b)
            if -0.9 < cos_theta < 0.9: continue

            distance = np.abs(a * mid_x_b + b * mid_y_b + c) / np.sqrt(a ** 2 + b ** 2)

            if 150 < distance < 250:
                cv2.line(merged_frame, (lines[rand[0]][0][0] + 10 * vec_a[0], lines[rand[0]][0][1] - 10 * vec_a[1]),
                         (lines[rand[0]][0][0] - 10 * vec_a[0], lines[rand[0]][0][1] + 10 * vec_a[1]), (0, 0, 255), 2)

                cv2.line(merged_frame, (lines[rand[1]][0][0] + 10 * vec_b[0], lines[rand[1]][0][1] - 10 * vec_b[1]),
                         (lines[rand[1]][0][0] - 10 * vec_b[0], lines[rand[1]][0][1] + 10 * vec_b[1]), (0, 0, 255), 2)

                bottom_interception_a = lines[rand[0]][0][0] + vec_a[0] * (lines[rand[0]][0][1] - 300) / vec_a[1]
                bottom_interception_b = lines[rand[1]][0][0] + vec_b[0] * (lines[rand[1]][0][1] - 300) / vec_b[1]
                bottom_interception_center = int((bottom_interception_a + bottom_interception_b) / 2)

                cv2.circle(merged_frame, (bottom_interception_center, 300), 10, (0, 255, 0), -1)
                interception = bottom_interception_center - 300

                a_temp_x = vec_a[0]
                a_temp_y = vec_a[1]
                vec_a_up = (100 * a_temp_x / magnitude_a, 100 * a_temp_y / magnitude_a) if a_temp_y >= 0 else (
                    -100 * a_temp_x / magnitude_a, -100 * a_temp_y / magnitude_a)

                b_temp_x = vec_b[0]
                b_temp_y = vec_b[1]
                vec_b_up = (100 * b_temp_x / magnitude_b, 100 * b_temp_y / magnitude_b) if b_temp_y >= 0 else (
                    -100 * b_temp_x / magnitude_b, -100 * b_temp_y / magnitude_b)

                vec_mid = (int((vec_a_up[0] + vec_b_up[0]) / 2), int((vec_a_up[1] + vec_b_up[1]) / 2))

                cv2.line(merged_frame, (bottom_interception_center + 10 * vec_mid[0], 300 - 10 * vec_mid[1]),
                         (bottom_interception_center - 10 * vec_mid[0], 300 + 10 * vec_mid[1]), (0, 255, 0), 2)
                angle = np.degrees(np.arctan(vec_mid[1] / vec_mid[0])) if vec_mid[0] != 0 else 90

                # 두 직선에 모두 직교하는 직선 찾기를 시도하고 있으면 그리기
                horizontal_line = None
                t2 = time.time()
                while True:
                    if time.time() - t2 > 0.01: break
                    rand2 = random.randint(0, len(lines) - 1)
                    for x1, y1, x2, y2 in lines[rand2]:
                        vec_h = (x2 - x1, y1 - y2)
                        magnitude_h = np.sqrt(vec_h[0] ** 2 + vec_h[1] ** 2)
                    if (np.abs(np.dot(vec_h, vec_a)) / (magnitude_a * magnitude_h)) < 0.2 and \
                            (np.abs(np.dot(vec_h, vec_b)) / (magnitude_b * magnitude_h)) < 0.2:
                        horizontal_line = lines[rand2][0]
                        vec_h = (horizontal_line[0] - horizontal_line[2], horizontal_line[3] - horizontal_line[1])
                        break

                if horizontal_line is not None:
                    if vec_a[1] < 0:
                        temp1, temp2 = vec_a[0] * -1, vec_a[1] * -1
                        vec_a = (temp1, temp2)

                    if vec_b[1] < 0:
                        temp1, temp2 = vec_b[0] * -1, vec_b[1] * -1
                        vec_b = (temp1, temp2)

                    horizontal_mid = (
                        (horizontal_line[0] + horizontal_line[2]) / 2, (horizontal_line[1] + horizontal_line[3]) / 2)
                    vertical_mid = ((lines[rand[0]][0][0] + lines[rand[0]][0][2]) / 2,
                                    (lines[rand[0]][0][1] + lines[rand[0]][0][3]) / 2)
                    vec_VtoH = (horizontal_mid[0] - vertical_mid[0], vertical_mid[1] - horizontal_mid[1])

                    if (np.dot(vec_a, vec_VtoH) > 0 and np.dot(vec_b, vec_VtoH) > 0):
                        cv2.line(merged_frame, (horizontal_line[0] + 10 * vec_h[0], horizontal_line[1] - 10 * vec_h[1]),
                                 (horizontal_line[0] - 10 * vec_h[0], horizontal_line[1] + 10 * vec_h[1]), (255, 0, 0),
                                 2)
                break

        return merged_frame, min_dist, interception, angle

    def stop_line_detection(self):
        merged_frame = self.make_merged_frame()
        # filtered_frame = cv2.Canny(merged_frame, 50, 100)
        filtered_frame = cv2.inRange(merged_frame, self.lower_white, self.upper_white)

        lines = cv2.HoughLinesP(filtered_frame, 1, np.pi / 360, 40, 100, 80)

        t1 = time.time()

        stop_line = None

        while True:
            if lines is None: break
            slope, length = 0, 0
            rand = random.randint(0, len(lines) - 1)

            for x1, y1, x2, y2 in lines[rand]:
                slope = ((y2 - y1) / (x1 - x2)) if (x1 != x2) else 10000
                length = np.sqrt((y2 - y1) ** 2 + (x1 - x2) ** 2)

            if abs(slope) < 0.05 and length > 40:
                stop_line = lines[rand]
                break

            if (time.time() - t1) >= 0.01:
                stop_line = None
                break

        distance = None

        if stop_line is not None:
            for x1, y1, x2, y2 in stop_line:
                a = ((y2 - y1) / (x1 - x2))
                b = 300 - y1 - a * x1
                distance = abs(300 * a + b) / np.sqrt(a ** 2 + 1)

                cv2.line(merged_frame, (x1 + 100 * (x1 - x2), y1 + 100 * (y2 - y1)),
                         (x1 - 100 * (x1 - x2), y1 - 100 * (y2 - y1)), (0, 0, 255), 2)

        # self.data.planner_monitoring_frame = (merged_frame, 600, 300)

        return merged_frame, distance

    def lane_detection(self):
        if self.data_source.mid_frame is None: return None
        temp_frame = self.data_source.mid_frame[290:448, 0:800].copy()
        edged = cv2.Canny(temp_frame, 50, 150)

        left_lines = []
        right_lines = []
        left_weights = []
        right_weights = []

        left_lane = None
        right_lane = None

        final_interception = None
        final_angle = None

        hough_lines = cv2.HoughLinesP(edged, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20, maxLineGap=500)

        for line in hough_lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2 or y1 == y2: continue

                slope = (y2 - y1) / (x2 - x1)
                length = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                ceiling_interception = int(x1 - (0 - y1) * (x2 - x1) / (y1 - y2))
                bottom_interception = int(x1 - (158 - y1) * (x2 - x1) / (y1 - y2))

                if 300 > ceiling_interception or ceiling_interception > 500 or bottom_interception < 0 or \
                        bottom_interception > 800 or np.abs(slope) < 0.1 or length < 200: continue

                if slope < 0:
                    left_lines.append((slope, bottom_interception))
                    left_weights.append((length))

                else:
                    right_lines.append((slope, bottom_interception))
                    right_weights.append((length))

                left_lane = np.dot(left_weights, left_lines) / np.sum(left_weights) if len(left_weights) > 0 else None
                right_lane = np.dot(right_weights, right_lines) / np.sum(right_weights) if len(right_weights) > 0 else None

        if left_lane is not None and right_lane is not None:
            left_pt1 = (int(left_lane[1]), 158)
            left_pt2 = (int(left_lane[1] - 158 / left_lane[0]), 0)
            right_pt1 = (int(right_lane[1]), 158)
            right_pt2 = (int(right_lane[1] - 158 / right_lane[0]), 0)
            mid_pt1 = (int((left_pt1[0] + right_pt1[0]) / 2), 158)
            mid_pt2 = (int((left_pt2[0] + right_pt2[0]) / 2), 0)

            cv2.line(temp_frame, left_pt1, left_pt2, (0, 0, 255), 10)
            cv2.line(temp_frame, right_pt1, right_pt2, (0, 0, 255), 10)
            cv2.line(temp_frame, mid_pt1, mid_pt2, (0, 255, 0), 10)

            mid_slope = 158 / (mid_pt2[0] - mid_pt1[0]) if mid_pt2[0] != mid_pt1[0] else 1000000
            mid_angle = np.degrees(np.arctan(mid_slope))
            if mid_angle < 0: mid_angle += 180

            final_interception = mid_pt1[0] - 400
            final_angle = mid_angle

        return temp_frame, final_interception, final_angle


if __name__ == "__main__":
    import threading
    import time
    from dummy_data_source import DummySource

    testData = Data()
    # ------------------- Dummy Data 사용 시 아래 코드를 활성화 ----------------------
    testDDS = DummySource('2018-11-04-16-21-59')
    testLC = LaneCam(testDDS)  # DummySource for test
    dummy_thread = threading.Thread(target=testDDS.main)
    dummy_thread.start()

    # ------------------- 센서 Data 사용 시 아래 코드를 활성화 ----------------------
    # testDS = Source(testData)
    # lidar_source_thread = threading.Thread(target=testDS.lidar_stream_main)
    # left_cam_source_thread = threading.Thread(target=testDS.left_cam_stream_main)
    # right_cam_source_thread = threading.Thread(target=testDS.right_cam_stream_main)
    # mid_cam_source_thread = threading.Thread(target=testDS.mid_cam_stream_main)

    # lidar_source_thread.start()
    # left_cam_source_thread.start()
    # right_cam_source_thread.start()
    # mid_cam_source_thread.start()
    # -------------------------------------------------------------------------------

    time.sleep(1)
    while True:
        testLC.lane_detection()
        if cv2.waitKey(1) & 0xff == ord('q'): break
