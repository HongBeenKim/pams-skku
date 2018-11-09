from subroutine import Subroutine
from data_class import Data
import cv2


class LaneCam(Subroutine):
    def __init__(self, data):
        super().__init__(data)
        self.left_cam = cv2.VideoCapture(1)
        self.left_cam.set(3, 800)
        self.left_cam.set(4, 448)


    def main(self):
        while True:
            _, left_frame = self.left_cam.read()
            cv2.imshow("left", left_frame)
            if cv2.waitKey(1) & 0xff == ord('q'): break


test_data = Data()
test_lane = LaneCam(test_data)
test_lane.main()
