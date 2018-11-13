import cv2
import numpy as np
import sys

sys.path.append(".")
sys.path.append("../test")
from subroutine import Subroutine
from data_source import Source
from data_class import Data


class LaneCam(Subroutine):
    camera_matrix_L = np.array([[474.383699, 0, 404.369647], [0, 478.128447, 212.932297], [0, 0, 1]])
    camera_matrix_R = np.array([[473.334870, 0, 386.312394], [0, 476.881433, 201.662339], [0, 0, 1]])
    distortion_coefficients_L = np.array([0.164159, -0.193892, -0.002730, -0.001859])
    distortion_coefficients_R = np.array([0.116554, -0.155379, -0.001045, -0.001512])

    pts1_L = np.float32([[0, 0], [0, 428], [780, 0], [780, 428]])
    pts2_L = np.float32([[0, 312], [440, 445], [537, 0], [562, 354]])
    pts1_R = np.float32([[0, 0], [0, 428], [780, 0], [780, 428]])
    pts2_R = np.float32([[5, 0], [-2, 384], [503, 324], [119, 471]])

    Bird_view_matrix_L = cv2.getPerspectiveTransform(pts1_L, pts2_L)
    Bird_view_matrix_R = cv2.getPerspectiveTransform(pts1_R, pts2_R)

    lower_white = np.array([187, 180, 160], dtype=np.uint8)
    upper_white = np.array([255, 254, 255], dtype=np.uint8)

    def __init__(self, data_source: Source, data):
        super().__init__(data)
        self.data_source = data_source

    def make_merged_frame(self):
        if self.data_source.left_frame is None or self.data_source.right_frame is None:
            return None
        left_frame = self.data_source.left_frame.copy()
        right_frame = self.data_source.right_frame.copy()
        undistorted_left = cv2.undistort(left_frame, self.camera_matrix_L,
                                         self.distortion_coefficients_L,None, None)[10:438, 10:790]
        undistorted_right = cv2.undistort(right_frame, self.camera_matrix_R,
                                         self.distortion_coefficients_R, None, None)[10:438, 10:790]




        #TODO: warpPerspective

    def stop_line_detection(self):
        pass

    def lane_detection(self):
        if self.data_source.mid_frame is None: return
        temp_frame = self.data_source.mid_frame[290:448, 0:800].copy()
        edged = cv2.Canny(temp_frame, 50, 150)

        # image, contours, hierachy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # cv2.drawContours(temp_frame, contours, -1, (0, 255, 0), 10)

        hough_lines = cv2.HoughLinesP(edged, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20,
                                      maxLineGap=300)

        if hough_lines is not None:
            for i in range(0, len(hough_lines)):
                for x1, y1, x2, y2 in hough_lines[i]:
                    if y1 == y2: continue
                    ceiling_interception = int(x1 - (0 - y1) * (x2 - x1) / (y1 - y2))
                    bottom_interception = int(x1 - (158 - y1) * (x2 - x1) / (y1 - y2))
                    if 266 > ceiling_interception or ceiling_interception > 532 or x1 == x2 or (abs(y1 - y2) / abs(x1 - x2)) < 0.1 or np.sqrt(
                            (x1 - x2) ** 2 + (y1 - y2) ** 2) < 50: continue
                    cv2.circle(temp_frame, (int(x1 - (0 - y1) * (x2 - x1) / (y1 - y2)), 0), 10, 255, -1)
                    cv2.line(temp_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

        #self.data.lane_value  # TODO: set lane values

        cv2.imshow('test', temp_frame)
        cv2.imshow('edged', edged)

    def main(self):
        while True:
            self.lane_detection()
            if cv2.waitKey(1) & 0xff == ord(' '): break


if __name__ == "__main__":
    import threading
    import time
    from dummy_data_source import DummySource

    testData = Data()
    testDS = Source()
    testDDS = DummySource('2018-11-04-17-01-16')
    testLC = LaneCam(testDDS, testData)  # DummySource for test

    # lidar_source_thread = threading.Thread(target=testDS.lidar_stream_main)
    # left_cam_source_thread = threading.Thread(target=testDS.left_cam_stream_main)
    # right_cam_source_thread = threading.Thread(target=testDS.right_cam_stream_main)
    # mid_cam_source_thread = threading.Thread(target=testDS.mid_cam_stream_main)

    dummy_thread = threading.Thread(target=testDDS.main)
    dummy_thread.start()

    # lidar_source_thread.start()
    # left_cam_source_thread.start()
    # right_cam_source_thread.start()
    # mid_cam_source_thread.start()

    time.sleep(2)

    testLC.main()
