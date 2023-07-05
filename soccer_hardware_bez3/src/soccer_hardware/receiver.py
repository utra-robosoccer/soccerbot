# game_controller_receiver.py

import struct
import time
from threading import Event, Lock, Thread

import rospy as rp
import serial
from transformations import *
from utility import log_string


class Receiver(Thread):
    def __init__(self, ser, group=None, target=None, name=None):
        super(Receiver, self).__init__(group=group, target=target, name=name)
        self._name = name
        self._stop_event = Event()
        self._num_rx = 0
        self._ser = ser
        self._num_rx_lock = Lock()
        self._timeout = 0.010  # 10 ms
        self._rate = 100
        self._imu_payload = np.ndarray(shape=(6, 1))
        self._angles_payload = np.ndarray(shape=(12, 1))

    def stop(self):
        """
        Causes the thread to exit after the next receive event (whether a
        success or a failure)
        """
        self._stop_event.set()

    def _stop_requested(self):
        return self._stop_event.is_set()

    def get_num_rx(self):
        """
        Returns the number of successful and failed receptions
        """
        with self._num_rx_lock:
            return self._num_rx

    def set_timeout(self, timeout):
        self._timeout = timeout

    def bind(self, callback):
        """
        Attaches a function to be called after a successful reception
        """
        self._callback = callback

    def run(self):
        """
        Reads packets from the microcontroller and sends the data up to the
        application through a callback function
        """
        log_string("Starting Rx thread ({0})...".format(self._name))

        MAX_T_EXCEPT_DENSITY = 10 # fails/s
        t_excepts_ = []

        r = rp.Rate(self._rate)
        while not rp.is_shutdown() and not self._stop_requested():
            try:
                (receive_succeeded, result) = self._receive_packet_from_mcu(self._timeout)
                if receive_succeeded:
                    self._callback(result)
                    with self._num_rx_lock:
                        self._num_rx = self._num_rx + 1
            except serial.serialutil.SerialException as e:
                print(e)
                log_string("Serial exception in thread {0}".format(self._name))

                t_excepts_.append(time.time())
                if len(t_excepts_) > 1 and 1 / np.mean(np.diff(np.array(t_excepts_))) > MAX_T_EXCEPT_DENSITY:
                    self._ser.reopen()
                    del t_excepts_[:]
            r.sleep()

        log_string("Stopping Rx thread ({0})...".format(self._name))
