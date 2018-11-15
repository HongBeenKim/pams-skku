import serial
import sys

sys.path.append(".")
from serial_packet import SerialPacket
from subroutine import Subroutine
from data_class import Data


class CarPlatform(Subroutine):
    def __init__(self, port: str, data: Data):
        super().__init__(data)
        self.init_error_flag = False  # False meaning start from test code
        try:
            self.serial = serial.Serial(port, 115200)
        except serial.serialutil.SerialException as e:
            print("ERROR!!! make sure your COMPORT: ", e)
            self.init_error_flag = True

    def main(self):
        if self.init_error_flag: return 1
        while True:
            self.receive()
            self.send()

            if self.data.start_from_main_flag and self.data.read_packet.aorm == SerialPacket.AORM_MANUAL:
                self.data.reset_to_default()

            if self.data.is_all_system_stop():
                break
        self.serial.close()
        return 0

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


if __name__ == "__main__":
    import time
    import threading


    def steer_left_test(test_data: Data):
        test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                    steer=SerialPacket.STEER_MAXLEFT, brake=SerialPacket.BRAKE_NOBRAKE)


    def steer_straight_test(test_data: Data):
        test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                    steer=SerialPacket.STEER_STRAIGHT, brake=SerialPacket.BRAKE_NOBRAKE)


    def steer_right_test(test_data: Data):
        test_data.set_control_value(gear=SerialPacket.GEAR_NEUTRAL, speed=SerialPacket.SPEED_MIN,
                                    steer=SerialPacket.STEER_MAXRIGHT, brake=SerialPacket.BRAKE_NOBRAKE)


    test_data = Data()
    test_platform = CarPlatform('COM5', test_data)  # PLEASE CHECK YOUR COMPORT
    platform_thread = threading.Thread(target=test_platform.main)
    platform_thread.start()

    if test_data.read_packet.aorm == SerialPacket.AORM_AUTO:
        test_data.detected_mission_number = 1
        test_data.current_mode = 1
        print("mission num: ", test_data.detected_mission_number, test_data.current_mode)
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
