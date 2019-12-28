#!/usr/bin/env python

import serial.tools.list_ports
import rospy
from communication import Communication
from utility import *

if __name__ == "__main__":
    rospy.init_node("soccer_hardware")
    port = rospy.get_param("~port", "/dev/ttyACM")
    baud = rospy.get_param("~baud", 230400)

    log_string("Connecting To Embedded system")
    log_string("\tPort: " + port)
    log_string("\tBaud rate: " + str(baud))

    # Try all ranges in port
    i = 0
    ser = serial.Serial()
    for i in range(0,10):
        try:
            ser = serial.Serial(port + str(i), baud, timeout=0)
            break
        except serial.serialutil.SerialException as e:
            pass

    if i == 9:
        rospy.logerr("No serial port found: " + port)
        exit(0)


    attempt = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            comm = Communication(ser)
            comm.run()
            break

        except serial.serialutil.SerialException as e:
            ser.close()
            log_string("Serial exception. " + e.strerror  + " Retrying...(attempt {0})".format(attempt))
            attempt = attempt + 1
            rate.sleep()