from serial_packet import SerialPacket


class Data(object):
    def __init__(self):
        self.read_packet = SerialPacket()
        self.write_packet = SerialPacket()
