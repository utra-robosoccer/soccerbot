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

MAX_CONSEC_EMPTY = 5
class MotorReceiver(Receiver):
    consec_empty = 0
    def _receive_packet_from_mcu(self, timeout):
        with self._ser.lock:  # NOTE: possibly replace with acquire-release with timeout
            valid, raw_pos = jx_servo_util.uart_transact(self._ser, [], CMDS.POSITION, RWS.READ, preflush=True)
            pos = {}
            if valid or True:
                for servo_idx, (p_valid, p) in raw_pos.items():
                    if p_valid:
                        pos[servo_idx] = unpack6(p[1:3], False) / 0xFFF * 180.0  # note: we use degrees for motor angles

        # seems a bit too low-level to put this authority here, but invalid packets are thrown out by `Receiver` immediately so this is the only place that knows to do so
        if len(raw_pos) == 0:
            self.consec_empty += 1
            if self.consec_empty > MAX_CONSEC_EMPTY:
                self.consec_empty = 0
                rp.logwarn('REOPENING SERIAL PORT %s' % self._ser.port)
                self._ser.reopen()
        rp.loginfo_throttle(1, pos)
        return (valid or True, pos)
