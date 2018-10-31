import serial
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
    test_data.set_control_value(gear=test_data.GEAR_NEUTRAL, speed=0, steer=-200, brake=0)


def steer_right_test(test_data: Data):
    test_data.set_control_value(gear=test_data.GEAR_NEUTRAL, speed=0, steer=200, brake=0)


if __name__ == "__main__":
    import time
    test_data = Data()
    test_platform = CarPlatform('COM5', test_data)
    test_platform.main()
    # TODO: 테스트 코드 만들고 테스트 해 보기
    while True:
        print(test_data.car_platform_status())
        steer_left_test(test_data)
        time.sleep(1)
        steer_right_test(test_data)
        time.sleep(1)
