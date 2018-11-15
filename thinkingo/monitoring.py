import cv2
import numpy as np
import sys

sys.path.append(".")
from subroutine import Subroutine
from data_class import Data
from serial_packet import SerialPacket

FONT_THICKNESS = 3


class Monitoring(Subroutine):
    def __init__(self, data: Data):
        super().__init__(data)
        self.canvas = np.zeros(shape=(600, 1200, 3), dtype=np.uint8)

    def main(self):
        while True:
            # TODO: 각 프레임이 어느 미션일 때 어떤 사이즈로 들어오는지 확인
            # TODO: 사이즈 고려해서 concatenate 하고 imshow
            car_frame = self.put_car_status_and_mode()  # 240 600
            merged_lane_cam = self.data.lane_cam_monitoring_frame



            if merged_lane_cam is not None:
                self.canvas = np.vstack((car_frame, merged_lane_cam))

            cv2.imshow('monitoring', car_frame)
            if cv2.waitKey(1) & 0xff == ord(' '):
                self.data.stop_thinkingo()
                break
        cv2.destroyAllWindows()

    def put_car_status_and_mode(self):
        frame = np.zeros((240, 600, 3), dtype=np.uint8)

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
        if steer == 0:
            steer_direction_string = 'Straight'

        steer_string = steer_direction_string + '{:5.2f}'.format(abs(steer)) + 'deg'

        brake_string = 'Brake' + '{:7.2f}'.format(brake)
        frame = cv2.putText(img=frame, text=gear_speed_string, org=(0, 50), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=steer_string, org=(0, 110), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text=brake_string, org=(0, 170), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 255, 255), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text='current', org=(0, 230), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(255, 200, 0), thickness=FONT_THICKNESS)
        frame = cv2.putText(img=frame, text='detected', org=(300, 230), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=2,
                            color=(0, 255, 255), thickness=FONT_THICKNESS)
        return frame


if __name__ == "__main__":
    import threading
    from data_source import Source
    from car_platform import CarPlatform
    from lane_cam import LaneCam

    test_data = Data()
    # test_data_source = Source(test_data)
    #
    # left_cam_thread = threading.Thread(target=test_data_source.left_cam_stream_main)
    # right_cam_thred = threading.Thread(target=test_data_source.right_cam_stream_main)
    # left_cam_thread.start()
    # right_cam_thred.start()

    car = CarPlatform('COM5', test_data)
    # lane_cam = LaneCam(test_data_source, test_data)
    monitoring = Monitoring(test_data)

    car_thread = threading.Thread(target=car.main)
    # lane_cam_thread = threading.Thread(target=lane_cam.main)
    monitor_thread = threading.Thread(target=monitoring.main)

    car_thread.start()
    monitor_thread.start()
