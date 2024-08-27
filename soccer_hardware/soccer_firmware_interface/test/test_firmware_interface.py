import os

import numpy as np

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
        print("angle:", angle)
        goalPositionsAllMotors = []
        goalPositionsAllMotors = [0xFF, 0xFF]
        for motorID in range(30):
            goalPositionsAllMotors.append(angle_lo)
            goalPositionsAllMotors.append(angle_hi)

        # 2 header + 18 in the array below
        #      header   header   motorID#0   motorID#1   motorID#2   motorID#3    motorID#4    motorID#5    motorID#6     motorID#7
        # Index: [0]     [1]     [2] [3]     [4] [5]     [6] [7]      [8] [9]     [10] [11]    [12] [13]    [14] [15]     [16] [17]
        # s.write([0xFF, 0xFF, angle_lo, angle_hi, angle_lo, angle_hi, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18])
        result = s.write(goalPositionsAllMotors)
        time.sleep(0.01)
        # for i in range(20):
        #     res = s.read()
        #     print((res))

        count_loop += 1
        print(count_loop / (time.time() - t_start))


def test_firmware_interface():
    rospy.init_node("firmware_interface")
    rospy.set_param("motor_mapping", os.path.dirname(os.path.realpath(__file__)) + "/../config/bez2.yaml")
    rospy.set_param("motor_types", os.path.dirname(os.path.realpath(__file__)) + "/../config/motor_types.yaml")

    f = FirmwareInterface()

    for i in range(50000):
        j = JointState()
        j.name = [
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_elbow",
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_elbow",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee",
            "right_ankle_pitch",
            "right_ankle_roll",
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
            "head_yaw",
            "head_pitch",
        ]
        # j.position = [math.sin(i / 180 * math.pi) * 0.1, math.cos(i / 180 * math.pi) * 0.1]

        if True:  # test
            ang = 0.0
        else:
            ang = abs(math.sin(i / 180 * math.pi) * 0.2)
        j.position = [ang] * 20

        # j.position[0] = ang
        # j.position[1] = ang

        j.header.stamp = rospy.Time.now()

        f.joint_command_callback(j)

        time.sleep(0.01)

def test_firmware_interface_single_motor_range(motor_name: str = "left_knee"):
    rospy.init_node("firmware_interface")
    rospy.set_param("motor_mapping", os.path.dirname(os.path.realpath(__file__)) + "/../config/bez2.yaml")
    rospy.set_param("motor_types", os.path.dirname(os.path.realpath(__file__)) + "/../config/motor_types.yaml")

    f = FirmwareInterface()
    motor_range = np.linspace(-np.pi, 0)
    for i in motor_range:
        j = JointState()
        j.name = [
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_elbow",
            "right_shoulder_pitch",
            "right_shoulder_roll",
            "right_elbow",
            "right_hip_yaw",
            "right_hip_roll",
            "right_hip_pitch",
            "right_knee",
            "right_ankle_pitch",
            "right_ankle_roll",
            "left_hip_yaw",
            "left_hip_roll",
            "left_hip_pitch",
            "left_knee",
            "left_ankle_pitch",
            "left_ankle_roll",
            "head_yaw",
            "head_pitch",
        ]

        j.position = [0.0] * 20
        j.position[j.name.index(motor_name)] = i



        j.header.stamp = rospy.Time.now()

        f.joint_command_callback(j)
        rospy.sleep(1/10.0)
        time.sleep(0.01)



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
