#!/usr/bin/env python

import serial.tools.list_ports
import rospy
from communication import Communication
from utility import *

if __name__ == "__main__":
    rospy.init_node("soccer_hardware")

    port = rospy.get_param("port", "/dev/ttyACM0")
    baud = rospy.get_param("baud", 230400)
    trajectory = rospy.get_param("trajectory", "standing.csv")
    step_is_on = rospy.get_param("step", False)
    wait_feedback_is_on = rospy.get_param("step", True)

    log_string("Connecting To Embedded system")
    log_string("\tPort: " + port)
    log_string("\tBaud rate: " + str(baud))

    ser = serial.Serial()
    attempt = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            ser = serial.Serial(port, baud, timeout=0)
            ser.open()
            comm = Communication(ser, trajectory, step_is_on, wait_feedback_is_on)
            comm.run()

        except serial.serialutil.SerialException as e:
            ser.close()
            if str(e).find("FileNotFoundError"):
                log_string("Port not found. Retrying...(attempt {0})".format(attempt))
            else:
                log_string("Serial exception. " + e.strerror  + " Retrying...(attempt {0})".format(attempt))
            attempt = attempt + 1
            rate.sleep()