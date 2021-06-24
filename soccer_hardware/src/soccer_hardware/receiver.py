# game_controller_receiver.py

import serial
import struct
import time
from threading import Thread, Event, Lock
from transformations import *
from utility import log_string
import rospy as rp


class Receiver(Thread):
    def __init__(self, ser, group=None, target=None, name=None):
        super(Receiver, self).__init__(group=group, target=target, name=name)
        self._name = name
        self._stop_event = Event()
        self._num_rx = 0
        self._ser = ser
        self._num_rx_lock = Lock()
        self._timeout = 0.010  # 10 ms
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

    def _decode(self, raw):
        """ Decodes raw bytes received from the microcontroller. As per the agreed
            upon protocol, the first 4 bytes are for a header while the remaining
            80 bytes contain floats for each motor.
        """
        motors = list()
        imu = list()
        for i in range(12):
            # Here, we only unpack for 12 motors since that's all we have connected
            # in our current setup
            motors.append(struct.unpack('<f', raw[4 + i * 4:8 + i * 4])[0])
        for i in range(6):
            # Unpack IMU Data
            imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
        return motors, imu

    def _receive_packet_from_mcu(self, timeout):
        """
        Receives 80 bytes of the MCU provided that there is a valid 4-byte
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.

        Arguments
        ---------
        timeout : float
            Amount of time to wait for a packet to arrive before abandoning the
            receive operation. This duration is specified in seconds (usually a
            decimal such as 0.010, i.e. 10 ms) and is always relative to the
            time the first byte of the packet is received
        """

        receive_succeeded = False

        total_bytes_read = 0
        start_seq_count = 0
        buff = bytes(''.encode())

        time_start = time.time()
        time_curr = time_start

        num_bytes_available = 0
        data_received = False
        while not rp.is_shutdown():
            # First, we wait until we have received some data. If data has
            # already been received, then we quit if the timeout has elapsed
            while ((num_bytes_available == 0) and
                   not (data_received and (time_curr - time_start >= timeout)) and
                   not self._stop_requested()):
                time.sleep(0.01)
                time_curr = time.time()
                num_bytes_available = self._ser.in_waiting
            if self._stop_requested() or ((num_bytes_available == 0) and
                                          (data_received and (time_curr - time_start >= timeout))):
                break
            else:
                data_received = True
                # If we receive some data, we process it here then go back to
                # waiting for more
                rawData = self._ser.read(num_bytes_available)
                for i in range(num_bytes_available):
                    if start_seq_count == 4:

                        buff = buff + rawData[i:i + 1]
                        total_bytes_read = total_bytes_read + 1

                        if total_bytes_read == 84:
                            # If we get here, we have received a full packet
                            receive_succeeded = True
                            break
                    else:
                        if struct.unpack('<B', rawData[i:i + 1])[0] == 0xFF:
                            start_seq_count = start_seq_count + 1
                        else:
                            start_seq_count = 0
                num_bytes_available = 0
                if receive_succeeded:
                    with self._num_rx_lock:
                        self._num_rx = self._num_rx + 1
                    break
        return receive_succeeded, buff

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
        try:
            while not rp.is_shutdown() and not self._stop_requested():
                (receive_succeeded, buff) = self._receive_packet_from_mcu(self._timeout)
                if receive_succeeded:
                    (recvAngles, recvIMUData) = self._decode(buff)
                    angle_array = np.array(recvAngles)
                    received_angles = angle_array[:, np.newaxis]
                    received_imu = np.array(recvIMUData).reshape((6, 1))
                    self._callback(received_angles, received_imu)
        except serial.serialutil.SerialException:
            log_string("Serial exception in thread {0}".format(self._name))
        log_string("Stopping Rx thread ({0})...".format(self._name))
