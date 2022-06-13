import struct
import time
from threading import Event, Lock, Thread

import rospy as rp
import serial
from motor_util import CMD_HEADS, CMDS, RWS, uart_transact
from receiver import Receiver
from transformations import *
from utility import log_string


class MotorReceiver(Receiver):
    def _receive_packet_from_mcu(self, timeout):
        with self._ser._motor_lock:  # NOTE: possibly replace with acquire-release with timeout
            valid, raw_pos = uart_transact(self._ser, [], CMDS.POSITION, RWS.READ, timeout)
            pos = {}
            if valid:
                # print('!!!!!!!RX!!!!!!!')
                # print(raw_pos)
                for servo_idx, (p_valid, p) in raw_pos.items():
                    if p_valid:
                        p = (p[1] & 0x3F) | ((p[2] & 0x3F) << 6)
                        if p & 0x800:
                            p -= 0x1000
                        pos[servo_idx] = float(p) / 0xFFF * 180.0  # note: we use degrees for motor angles

        return (valid, pos)
