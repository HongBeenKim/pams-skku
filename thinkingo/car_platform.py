import serial
from serial_packet import SerialPacket
from subroutine import Subroutine
from data_class import Data


class CarPlatform(Subroutine):
    def __init__(self, port: str, data: Data):
        super().__init__(data)
        try:
            self.serial = serial.Serial(port, 115200)
        except Exception as e:
            print("car_platform INIT ERROR: ", e)

    def main(self):
        while True:
            self.receive()
            self.send()

    def receive(self):
        try:
            binary_message = self.serial.read(18)
        except Exception as e:
            print("car_platform RECEIVE ERROR: ", e)
            return
        self.data.read_packet.read_bytes(binary_message)

    def send(self):
        self.data.write_packet.alive = self.data.read_packet.alive
        try:
            self.serial.write(self.data.write_packet.write_bytes())
        except Exception as e:
            print("car_platform SEND ERROR: ", e)


def steer_left_test(test_data: Data):
    test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                steer=SerialPacket.STEER_MAXLEFT, brake=SerialPacket.BRAKE_NOBRAKE)


def steer_straight_test(test_data: Data):
    test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                steer=SerialPacket.STEER_STRAIGHT, brake=SerialPacket.BRAKE_NOBRAKE)


def steer_right_test(test_data: Data):
    test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                steer=SerialPacket.STEER_MAXRIGHT, brake=SerialPacket.BRAKE_NOBRAKE)


if __name__ == "__main__":
    import time
    import threading

    test_data = Data()
    test_platform = CarPlatform('COM5', test_data)
    platform_thread = threading.Thread(target=test_platform.main)
    platform_thread.start()

    # TODO: 테스트 되게 만들기 (시리얼 케이블 문제로 추정)
    if test_data.read_packet.aorm == SerialPacket.AORM_AUTO:
        t = time.time()
        i = 1
        while True:
            print("read: ", test_data.car_platform_status())
            print("WRITE: ", test_data.write_packet.steer)
            if time.time() - t < 2:
                if i == 1:
                    steer_right_test(test_data)
                else:
                    steer_left_test(test_data)
            else:
                t = time.time()
                if i == 1:
                    i = 0
                else:
                    i = 1
