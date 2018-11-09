from subroutine import Subroutine
from dummy_data_source import DummySource
import cv2

class MotionPlanner(Subroutine):
    def __init__(self, dummy_data: DummySource):
        self.data = dummy_data

    def testing(self):
        while True:
            temp_frame = self.data.mid_frame[290:448, 0:800]
            edged = cv2.Canny(temp_frame, 100, 200)

            image, contours, hierachy = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            cv2.drawContours(temp_frame, contours, -1, (0, 255, 0), 10)

            cv2.imshow('test', temp_frame)
            cv2.imshow('edged', edged)
            if cv2.waitKey(1) & 0xff == ord(' '): break


if __name__ == "__main__":
    dmsc = DummySource('2018-11-04-17-01-16')
    dmsc.start()
    testmot = MotionPlanner(dmsc)
    testmot.testing()
