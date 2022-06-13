import struct
import time
from threading import Event, Lock, Thread

import rospy as rp
import serial
from receiver import Receiver
from transformations import *
from utility import log_string

NAN_HEAD = b"\x00\x00\xc0\x7f"
# looking specifically for the NAN constant from STM32 C libs
# generalized IEEE754 NaN is all-1s exponent with any non-zero significand
# so math.isnan is too porous a filter


class IMUReceiver(Receiver):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._working_buf = b""

    def _receive_packet_from_mcu(self, timeout):
        self._ser.timeout = timeout
        self._working_buf += self._ser.read(28)

        D = self._working_buf.split(NAN_HEAD)
        ret = (False, None)
        if len(D) > 0:
            # if len(D[-1]) >= 24: # clear the buffer if it's readable
            # 	self._working_buf = b''
            # else:
            # 	self._working_buf = D[-1]

            for d in D[::-1]:
                if len(d) >= 24:
                    ret = (True, struct.unpack("<" + "f" * 6, d[:24]))
                    break
            self._working_buf = D[-1]

        return ret
