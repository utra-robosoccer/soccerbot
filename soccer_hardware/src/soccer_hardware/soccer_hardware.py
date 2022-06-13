#!/usr/bin/env python3

import rospy
import serial.tools.list_ports
from communication import Communication
from utility import *

if __name__ == "__main__":
    rospy.init_node("soccer_hardware")
    imu_port = rospy.get_param("~imu_port", "/dev/ttyACM")
    servo_port = rospy.get_param("~servo_port", "/dev/ttyUSB")
    imu_baud = rospy.get_param("~imu_baud", 230400)
    servo_baud = rospy.get_param("~servo_baud", 1000000)

    log_string("Connecting To Embedded system")
    log_string("\tServo Port: " + servo_port)
    log_string("\tServo Baud rate: " + str(servo_baud))
    log_string("\tIMU Port: " + imu_port)
    log_string("\tIMU Baud rate: " + str(imu_baud))

    # Try all ranges in port
    imu_ser = None
    servo_ser = None
    for i in range(0, 10):
        try:
            imu_ser = serial.Serial(imu_port + str(i), imu_baud, timeout=0)
        except serial.serialutil.SerialException as e:
            pass

        try:
            servo_ser = serial.Serial(servo_port + str(i), servo_baud, timeout=0)
        except serial.serialutil.SerialException as e:
            pass

    if imu_ser is None or servo_ser is None:
        rospy.logerr(
            "No serial port found: "
            + ("IMU port `%s`" % imu_port if imu_ser is None else "")
            + (" and " if imu_ser is None and servo_ser is None else "")
            + ("Servo port `%s`" % servo_port if servo_ser is None else "")
        )
        exit(0)

    attempt = 0
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        try:
            comm = Communication(servo_ser, imu_ser)
            comm.run()
            break

        except serial.serialutil.SerialException as e:
            ser.close()
            log_string("Serial exception. " + e.strerror + " Retrying...(attempt {0})".format(attempt))
            attempt = attempt + 1
            rate.sleep()
