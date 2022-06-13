# Transmitter.py

import struct
from queue import Queue
from threading import Event, Lock, Thread

import numpy as np
import rospy as rp
import serial
from motor_util import CMD_HEADS, CMDS, RWS, uart_transact
from utility import log_string

ANGLE2POT_VOLTS = 0xFFF / 180.0


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
        """Sends bytes to the MCU with the header sequence attached."""
        with self._ser._motor_lock:  # NOTE: possibly replace with acquire-release with timeout
            # print(byteStream)
            return uart_transact(self._ser, byteStream, CMDS.POSITION, RWS.WRITE, 0.01)

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
            while not rp.is_shutdown():
                while not self._cmd_queue.empty():
                    goal_angles = self._cmd_queue.get()
                    print(goal_angles)
                    self._send_packet_to_mcu((np.array(goal_angles) * ANGLE2POT_VOLTS).astype(np.uint16))
                    with self._num_tx_lock:
                        self._num_tx = self._num_tx + 1
        except serial.serialutil.SerialException:
            log_string("Serial exception in thread {0}".format(self._name))
        log_string("Stopping Tx thread ({0})...".format(self._name))
