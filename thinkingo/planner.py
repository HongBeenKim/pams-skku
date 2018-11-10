from subroutine import Subroutine
from dummy_data_source import DummySource
import cv2
import numpy as np


class MotionPlanner(Subroutine):
    def __init__(self, dummy_data: DummySource):
        self.data = dummy_data

    def testing(self):
        while True:
            temp_frame = self.data.mid_frame[290:448, 0:800]
            edged = cv2.Canny(temp_frame, 100, 200)

            #image, contours, hierachy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            #cv2.drawContours(temp_frame, contours, -1, (0, 255, 0), 10)

            hough_lines = cv2.HoughLinesP(edged, rho=1, theta=np.pi / 180, threshold=20, minLineLength=40, maxLineGap=300)

            if hough_lines is not None:
                for i in range(0, len(hough_lines)):
                    for x1, y1, x2, y2 in hough_lines[i]:
                        if x1 == x2: continue
                        if (abs(y1 - y2) / abs(x1 - x2)) < 0.6 or np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) < 100: continue
                        cv2.circle(temp_frame, (int(x1 - (158-y1)*(x2-x1)/(y1-y2)), 158), 10, 255, -1)
                        cv2.line(temp_frame, (x1, y1), (x2, y2), (0, 0, 255), 3)


            cv2.imshow('test', temp_frame)
            cv2.imshow('edged', edged)
            if cv2.waitKey(1) & 0xff == ord(' '): break


if __name__ == "__main__":
    dmsc = DummySource('2018-11-04-17-01-16')
    dmsc.start()
    testmot = MotionPlanner(dmsc)
    testmot.testing()
