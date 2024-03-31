import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import importlib
import logging
import math
import time

import rosparam
import rospy
import serial
import yaml
from sensor_msgs.msg import JointState
from soccer_firmware_interface.firmware_interface import FirmwareInterface


def test_send_command():
    s = serial.Serial("/dev/ttyACM1")
    angle = 200
    count_loop = 0
    t_start = time.time()
    while True:
        angle += 1
        angle %= 0x3FF
        angle_lo = angle & 0xFF
        angle_hi = (angle >> 8) & 0xFF
        # print('angle:', angle)
        s.write([0xFF, 0xFF, angle_lo, angle_hi, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])

        for i in range(20):
            res = s.read()
            print((res))

        count_loop += 1
        print(count_loop / (time.time() - t_start))


def test_firmware_interface():
    rospy.init_node("test")

    importlib.reload(logging)

    with open((os.path.dirname(os.path.abspath(__file__)) + "/../../config/motor_types.yaml")) as f:
        param_info = yaml.safe_load(f)
        rosparam.upload_params("motor_types", param_info)

    with open((os.path.dirname(os.path.abspath(__file__)) + "/../../config/bez2.yaml")) as f:
        param_info = yaml.safe_load(f)
        rosparam.upload_params("motor_mapping", param_info)

    f = FirmwareInterface()

    for i in range(50000):
        j = JointState()
        j.name = [
            "head_motor_0",
            "head_motor_1",
            "left_arm_motor_0",
            "left_arm_motor_1",
            "right_arm_motor_0",
            "right_arm_motor_1",
            "left_leg_motor_0",
            "left_leg_motor_1",
            "left_leg_motor_2",
            "left_leg_motor_3",
            "left_leg_motor_4",
            "left_leg_motor_5",
            "right_leg_motor_0",
            "right_leg_motor_1",
            "right_leg_motor_2",
            "right_leg_motor_3",
            "right_leg_motor_4",
            "right_leg_motor_5",
        ]
        # j.position = [math.sin(i / 180 * math.pi) * 0.1, math.cos(i / 180 * math.pi) * 0.1]
        # ang = math.sin(i / 180 * math.pi) * 0.2
        ang = 0.0
        j.position = [ang] * 18
        j.header.stamp = rospy.Time.now()

        f.joint_command_callback(j)

        # time.sleep(0.01)

    pass


def test_firmware_interface_normal():
    rospy.init_node("firmware_interface")
    rospy.set_param("motor_mapping", os.path.dirname(os.path.realpath(__file__)) + "/../../config/bez2.yaml")
    rospy.set_param("motor_types", os.path.dirname(os.path.realpath(__file__)) + "/../../config/motor_types.yaml")

    rospy.loginfo("Initializing Soccer Firmware")
    f = FirmwareInterface()
    rospy.loginfo("Starting Firmware")
    try:
        rospy.spin()
    except rospy.exceptions.ROSException as ex:
        exit(0)
