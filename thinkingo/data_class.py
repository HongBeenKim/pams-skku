from serial_packet import SerialPacket


class Data(object):
    def __init__(self):
        self._read_packet = SerialPacket()
        self._write_packet = SerialPacket()

    @property
    def read_packet(self):
        return self._read_packet

    @read_packet.setter
    def read_packet(self, read_packet):
        self._read_packet = read_packet

    @property
    def write_packet(self):
        return self._write_packet

    @write_packet.setter
    def write_packet(self, write_packet):
        self._write_packet = write_packet
