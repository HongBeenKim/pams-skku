import cv2
import numpy as np
import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data
from serial_packet import SerialPacket

FONT_THICKNESS = 2


class Monitoring(Subroutine):
    def __init__(self, data: Data):
        super().__init__(data)
        self.canvas = np.zeros(shape=(600, 1200, 3), dtype=np.uint8)

    def main(self):
        while True:
            self.canvas = self.put_car_platform_status()
            # sign cam
            # lane cam mid
            # lane cam side
            # lidar
            cv2.imshow('monitor', self.canvas)
            if cv2.waitKey(1) & 0xff == ord(' '):
                self.data.stop_thinkingo()
                break
        cv2.destroyAllWindows()

    def put_car_platform_status(self):
        frame = np.zeros((180, 600, 3), dtype=np.uint8)

        gear, speed, steer, brake, aorm, alive, enc = self.data.car_platform_status()
        gear_string = ''
        if gear == SerialPacket.GEAR_FORWARD:
            gear_string = 'Drive'
        elif gear == SerialPacket.GEAR_NEUTRAL:
            gear_string = 'Neutral'
        elif gear == SerialPacket.GEAR_BACKWARD:
            gear_string = 'Rear'

        gear_speed_string = gear_string + '{:8.2f}'.format(speed) + 'kph'
        steer_direction_string = 'Left    ' if steer > 0 else 'Right   '
        if steer == 0: steer_direction_string = 'Straight'

        steer_string = steer_direction_string + '{:5.2f}'.format(abs(steer)) + 'deg'

        brake_string = 'Brake' + '{:7.2f}'.format(brake)
        frame = cv2.putText(img=frame, text=gear_speed_string, org=(0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=steer_string, org=(0, 110), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=brake_string, org=(0, 170), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        return frame


if __name__ == "__main__":
    import threading
    from car_platform import CarPlatform

    test_data = Data()

    car = CarPlatform('COM5', test_data)
    monitoring = Monitoring(test_data)

    car_thread = threading.Thread(target=car.main)
    monitor_thread = threading.Thread(target=monitoring.main)

    car_thread.start()
    monitor_thread.start()
