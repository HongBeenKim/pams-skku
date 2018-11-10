from subroutine import Subroutine
from dummy_data_source import DummySource
import cv2
import numpy as np
from data_class import Data


class LaneCam(Subroutine):
    camera_matrix_L = np.array([[474.383699, 0, 404.369647], [0, 478.128447, 212.932297], [0, 0, 1]])
    distortion_coefficients_L = np.array([0.164159, -0.193892, -0.002730, -0.001859])
    pts1_L = np.float32([[0, 0], [0, 428], [780, 0], [780, 428]])
    pts2_L = np.float32([[0, 312], [440, 445], [537, 0], [562, 354]])
    Bird_view_matrix_L = cv2.getPerspectiveTransform(pts1_L, pts2_L)

    lower_white = np.array([187, 180, 160], dtype=np.uint8)
    upper_white = np.array([255, 254, 255], dtype=np.uint8)

    def __init__(self, data_source: DummySource, data):
        super().__init__(data)
        self.data_source = data_source


    def lane_detection(self):
        temp_frame = self.data_source.mid_frame[290:448, 0:800]
        edged = cv2.Canny(temp_frame, 50, 150)

        # image, contours, hierachy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # cv2.drawContours(temp_frame, contours, -1, (0, 255, 0), 10)

        hough_lines = cv2.HoughLinesP(edged, rho=1, theta=np.pi / 180, threshold=20, minLineLength=20,
                                      maxLineGap=300)

        if hough_lines is not None:
            for i in range(0, len(hough_lines)):
                for x1, y1, x2, y2 in hough_lines[i]:
                    if x1 == x2: continue
                    if (abs(y1 - y2) / abs(x1 - x2)) < 0.1 or np.sqrt(
                        (x1 - x2) ** 2 + (y1 - y2) ** 2) < 50: continue
                    cv2.circle(temp_frame, (int(x1 - (158 - y1) * (x2 - x1) / (y1 - y2)), 158), 10, 255, -1)
                    cv2.line(temp_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)

        cv2.imshow('test', temp_frame)
        cv2.imshow('edged', edged)

    def main(self):
        while True:
            self.lane_detection()
            if cv2.waitKey(1) & 0xff == ord(' '): break


if __name__ == "__main__":
    import threading
    import time

    testData = Data()
    testDS = DummySource('2018-11-04-17-01-16')
    testLC = LaneCam(testDS, testData)

    stream_thread = threading.Thread(target=testDS.main)
    stream_thread.start()
    time.sleep(1)
    testLC.main()

