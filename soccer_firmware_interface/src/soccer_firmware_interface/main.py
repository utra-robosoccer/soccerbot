#!/usr/bin/env python3
import os

import rospy
import serial

from soccer_firmware_interface.firmware_interface import FirmwareInterface

if __name__ == "__main__":
    rospy.init_node("firmware_interface")

    f = FirmwareInterface()

    rospy.loginfo("Initializing Soccer Firmware")
    rospy.loginfo("Starting Firmware")
    try:
        rospy.spin()
    except rospy.exceptions.ROSException as ex:
        exit(0)
