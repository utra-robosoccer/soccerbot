import struct
import time
from threading import Event, Lock, Thread

import jx_servo_util
import rospy as rp
import serial
from common_motor_util import unpack6
from jx_servo_util import CMD_HEADS, CMDS, RWS
from receiver import Receiver
from transformations import *
from utility import log_string


class MotorReceiver(Receiver):
    def _receive_packet_from_mcu(self, timeout):
        with self._ser._motor_lock:  # NOTE: possibly replace with acquire-release with timeout
            valid, raw_pos = jx_servo_util.uart_transact(self._ser, [], CMDS.POSITION, RWS.READ, timeout)
            pos = {}
            if valid:
                # print('!!!!!!!RX!!!!!!!')
                for servo_idx, (p_valid, p) in raw_pos.items():
                    if p_valid:
                        pos[servo_idx] = unpack6(p[1:3], False) / 0xFFF * 180.0  # note: we use degrees for motor angles

        return (valid, pos)
