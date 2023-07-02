#!/usr/bin/env python3
import time

import rospy
import serial

from soccer_firmware_interface.firmware_interface import FirmwareInterface

# def run():
#     s = serial.Serial("/dev/ttyACM1")
#     angle = 200
#     count_loop = 0
#     t_start = time.time()
#     while True:
#         angle += 1
#         angle %= 0xFF
#         # print('angle:', angle)
#         s.write([0xFF, 0xFF, (angle >> 8), angle, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
#
#         for i in range(20):
#             res = s.read()
#             print((res))
#
#         count_loop += 1
#         print(count_loop / (time.time() - t_start))


if __name__ == "__main__":
    rospy.init_node("firmware_interface")

    f = FirmwareInterface()

    rospy.loginfo("Initializing Soccer Firmware")
    rospy.loginfo("Starting Firmware")
    try:
        rospy.spin()
    except rospy.exceptions.ROSException as ex:
        exit(0)
