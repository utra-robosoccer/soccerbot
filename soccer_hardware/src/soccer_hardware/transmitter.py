# Transmitter.py

import serial
import struct
from threading import Thread, Event, Lock
from Queue import Queue
from transformations import *
from utility import log_string


class Transmitter(Thread):
    def __init__(self, ser, dry_run=False, group=None, target=None, name=None):
        super(Transmitter, self).__init__(group=group, target=target, name=name)
        self._name = name
        self._cmd_queue = Queue(1)
        self._stop_event = Event()
        self._dry_run = dry_run
        self._ser = ser
        self._num_tx = 0
        self._num_tx_lock = Lock()

    def stop(self):
        """
        Prevents any more commands from being added to the queue and causes the
        thread to exit once the queue is empty.
        """
        self._stop_event.set()

    def _stopped(self):
        return self._stop_event.is_set()

    def _send_packet_to_mcu(self, byteStream):
        """ Sends bytes to the MCU with the header sequence attached.
        """
        header = struct.pack('<L', 0xFFFFFFFF)
        id = struct.pack('<I', 0x1234)
        padding = bytes(''.encode())
        footer = struct.pack('<L', 0x00000000)

        num_bytes = len(byteStream)
        if num_bytes < 80:
            padding = struct.pack('<B', 0x00) * (80 - num_bytes)

        packet = header + id + byteStream + padding + footer
        if self._dry_run:
            print(str(packet) + "\n")
        else:
            self._ser.write(packet)

    def _vec2bytes(self, vec):
        """ Transforms a numpy vector to a byte array, with entries interpreted as
            32-bit floats.
        """
        byte_arr = bytes(''.encode())
        for element in vec:
            byte_arr = byte_arr + struct.pack('f', element)
        return byte_arr

    def get_num_tx(self):
        with self._num_tx_lock:
            return self._num_tx

    def send(self, goal_angles):
        """
        Adds a set of goal angles to the command queue.
        """
        if not self._stopped():
            self._cmd_queue.put(goal_angles)

    def run(self):
        """
        Services the command queue; sends packets to the microcontroller.
        """
        log_string("Starting Tx thread ({0})...".format(self._name))
        try:
            while True:
                if self._stopped() and self._cmd_queue.empty():
                    break
                while not self._cmd_queue.empty():
                    cmd = self._cmd_queue.get()
                    goal_angles = ctrlToMcuAngles(cmd)
                    self._send_packet_to_mcu(self._vec2bytes(goal_angles))
                    with self._num_tx_lock:
                        self._num_tx = self._num_tx + 1
        except serial.serialutil.SerialException:
            log_string("Serial exception in thread {0}".format(self._name))
        log_string("Stopping Tx thread ({0})...".format(self._name))
