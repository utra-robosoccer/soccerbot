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

    def start(self, *args, **kwargs):
        with self._ser._motor_lock:
            uart_transact(self._ser, [1600, 1, 15] * 13, CMDS.PID_COEFF, RWS.WRITE)  # push initial PID gains
            uart_transact(self._ser, [1700] * 13, CMDS.MAX_DRIVE, RWS.WRITE)  # push initial maximum drive (out of 4096)
        super().start(*args, **kwargs)

    def stop(self):
        """
        Prevents any more commands from being added to the queue and causes the
        thread to exit once the queue is empty.
        """
        self._stop_event.set()

    def _stopped(self):
        return self._stop_event.is_set()

    def _send_packet_to_mcu(self, goal_angles):
        """Sends bytes to the MCU with the header sequence attached. Angles in degrees."""
        with self._ser._motor_lock:  # NOTE: possibly replace with acquire-release with timeout
            # print('**************', goal_angles)
            flat_goal_angles = [0] * (max(goal_angles.keys()) + 1)
            for idx, v in goal_angles.items():
                flat_goal_angles[idx] = int(v * ANGLE2POT_VOLTS)
        print(flat_goal_angles)
        return uart_transact(self._ser, flat_goal_angles, CMDS.POSITION, RWS.WRITE, 0.01)

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
                    # print(goal_angles)
                    self._send_packet_to_mcu(goal_angles)
                    with self._num_tx_lock:
                        self._num_tx = self._num_tx + 1
        except serial.serialutil.SerialException as e:
            log_string("Serial exception in thread {0}".format(self._name))
            print(e)
        log_string("Stopping Tx thread ({0})...".format(self._name))
