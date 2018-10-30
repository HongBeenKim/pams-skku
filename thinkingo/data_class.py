from serial_packet import SerialPacket


class Data(object):
    def __init__(self):
        self._read_packet = SerialPacket()
        self._write_packet = SerialPacket()

    @property
    def read_packet(self):
        return self._read_packet

    @property
    def write_packet(self):
        return self._write_packet

    def set_control_value(self, gear, speed, steer, brake):
        self._write_packet.gear = gear
        self._write_packet.speed = speed
        self._write_packet.steer = steer
        self._write_packet.brake = brake

    # TODO: 주은: 준혁아 엔코더 값 필요하면 말해라.
    def car_platform_status(self):
        gear = self._read_packet.gear
        speed = self._read_packet.speed / 10
        steer = self._read_packet.steer / 71
        brake = self._read_packet.brake / 200
        return gear, speed, steer, brake
