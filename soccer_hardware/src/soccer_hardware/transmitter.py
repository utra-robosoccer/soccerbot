# Transmitter.py

import serial
import struct
from threading import Thread, Event, Lock
from Queue import Queue
from transformations import *
from utility import logString


class Tx(Thread):
    def __init__(self, ser, dryrun=False, group=None, target=None, name=None):
        super(Tx, self).__init__(group=group, target=target, name=name)
        self._name = name
        self._cmd_queue = Queue(1)
        self._stop_event = Event()
        self._dryrun = dryrun
        self._ser = ser
        self._num_tx = 0
        self._num_tx_lock = Lock()

    def stop(self):
        '''
        Prevents any more commands from being added to the queue and causes the
        thread to exit once the queue is empty.
        '''
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

        numBytes = len(byteStream)
        if (numBytes < 80):
            padding = struct.pack('<B', 0x00) * (80 - numBytes)

        packet = header + id + byteStream + padding + footer
        if self._dryrun:
            print(str(packet) + "\n")
        else:
            self._ser.write(packet)

    def _vec2bytes(self, vec):
        """ Transforms a numpy vector to a byte array, with entries interpreted as
            32-bit floats.
        """
        byteArr = bytes(''.encode())
        for element in vec:
            byteArr = byteArr + struct.pack('f', element)
        return byteArr

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
        logString("Starting Tx thread ({0})...".format(self._name))
        try:
            while (1):
                if self._stopped() and self._cmd_queue.empty():
                    break
                while not self._cmd_queue.empty():
                    cmd = self._cmd_queue.get()
                    goal_angles = ctrlToMcuAngles(cmd)
                    self._send_packet_to_mcu(self._vec2bytes(goal_angles))
                    with self._num_tx_lock:
                        self._num_tx = self._num_tx + 1
        except serial.serialutil.SerialException as e:
            logString("Serial exception in thread {0}".format(self._name))
        logString("Stopping Tx thread ({0})...".format(self._name))
        return


if __name__ == "__main__":
    tx_thread = Tx(name="tx_th", ser="", dryrun=True)
    tx_thread.start()
    angles = np.zeros((18, 1))
    for i in range(20):
        tx_thread.send(angles)
    tx_thread.stop()
    tx_thread.join()
    print("Stopping main")