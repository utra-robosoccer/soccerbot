# transmitter.py

import struct
from transformations import *


class Transmitter:
    def __init__(self, ser):
        self.ser = ser
        self.num_transmissions = 0

    def change_port(self, ser):
        self.ser = ser

    def send_packet_to_mcu(self, byteStream):
        """ Sends bytes to the MCU with the header sequence attached.
        """
        header = struct.pack('<L', 0xFFFFFFFF)
        id = struct.pack('<I', 0x1234)
        padding = bytes(''.encode())
        footer = struct.pack('<L', 0x00000000)

        numBytes = len(byteStream)
        if numBytes < 80:
            padding = struct.pack('<B', 0x00) * (80 - numBytes)

        self.ser.write(header + id + byteStream + padding + footer)

    def vec2bytes(self, vec):
        """ Transforms a numpy vector to a byte array, with entries interpreted as
            32-bit floats.
        """
        byteArr = bytes(''.encode())
        for element in vec:
            byteArr = byteArr + struct.pack('f', element)
        return byteArr

    def transmit(self, goal_angles):
        """
        Converts the motor array from the coordinate system used by controls to
        that used by embedded and sends it to the MCU over serial
        """
        goal_angles = ctrlToMcuAngles(goal_angles)

        self.send_packet_to_mcu(self.vec2bytes(goal_angles))
        self.num_transmissions = self.num_transmissions + 1
