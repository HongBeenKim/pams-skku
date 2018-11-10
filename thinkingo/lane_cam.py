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
        self.data_source.start()

        while True:
            origin = self.data_source.left_frame
            # edged = cv2.Canny(origin, 100, 200)
            #
            # image, contours, hierachy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            # cv2.drawContours(origin, contours, -1, (0, 255, 0), 10)

            undistorted_left = cv2.undistort(origin, self.camera_matrix_L,
                                             self.distortion_coefficients_L, None, None)[10:438, 10:790]
            dst = cv2.warpPerspective(undistorted_left, self.Bird_view_matrix_L, (562, 445))

            #dst = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            dst2 = cv2.inRange(dst, self.lower_white, self.upper_white)
            cv2.imshow('origin', dst)
            cv2.imshow('left', dst2)

            if cv2.waitKey(1) & 0xff == ord(' '):
                break


if __name__ == "__main__":
    testData = Data()
    dmsc = DummySource('2018-11-04-15-56-04')
    testLC = LaneCam(dmsc, testData)
    testLC.lane_detection()
