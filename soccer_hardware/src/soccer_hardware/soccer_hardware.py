#!/usr/bin/env python
"""
Created June 6 2018

Author: Tyler and Jason
"""

import sys
import time
from comm import Comm
from utility import *


def main():
    args = parse_args()

    ros_is_on = args['ros']
    port = args['port']
    baud = args['baud']
    traj = args['traj']
    step_is_on = args['step']
    wait_feedback_is_on = args['use_wait_feedback']

    logString(list_ports())

    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))

    num_tries = 0
    comm = Comm()
    ser = serial.Serial(port, baud, timeout=0)
    first = True
    try:
        while True:
            try:
                logString("Connected")
                if not first:
                    ser.open()
                first = False
                comm.init(ser, ros_is_on, traj, step_is_on, wait_feedback_is_on)
                comm.begin_event_loop()
            except serial.serialutil.SerialException as e:
                comm.cleanup()
                ser.close()
                if (num_tries % 10 == 0):
                    if (str(e).find("FileNotFoundError")):
                        logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                    else:
                        logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
                time.sleep(0.1)
                num_tries = num_tries + 1
    except KeyboardInterrupt as e:
        logString("Interrupted main: {0}".format(repr(e)))
        comm.cleanup()
        ser.close()
        return


if __name__ == "__main__":
    main()
    sys.exit(0)