import math
import queue
from copy import deepcopy
from os.path import expanduser
from typing import Union

import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb
import rospy
import scipy
from rospy import ROSException
from scipy.signal import butter
from sensor_msgs.msg import JointState
from soccer_pycontrol.calibration import adjust_navigation_transform
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.path.path_robot import PathRobot

from soccer_common.pid import PID
from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi


class Soccerbot:
    """
    The main class for soccerbot, which receives and sends information to pybullet, inherited by ROS
    """

    def __init__(self, pose: Transformation, useFixedBase=False, useCalibration=True):
        """
        Initialization function for soccerbot. Does a series of calculations based on the URDF file for standing, walking poses

        :param pose: The position to initialize the robot, Z doesn't matter here as it will be set automatically
        :param useFixedBase: Whether to fix the base_link in the air for movement testing
        :param useCalibration: Whether to use calibration for walking path calculations
        """
        # TODO remove ros dependencies maybe a yaml file to read from

        rospy.loginfo("List of joints for loaded URDF")
        rospy.loginfo([pb.getJointInfo(self.body, i)[1] for i in range(pb.getNumJoints(self.body))])

        # For head rotation
        self.head_step = 0.0

        self.get_ready_rate = rospy.get_param("get_ready_rate", 0.02)

    def apply_head_rotation(self):
        """
        Does head rotation for the robot, robot will try to face the ball if it is ready, otherwise rotate around
        if its relocalizing or finding the ball
        """
        pass

    def apply_foot_pressure_sensor_feedback(self, floor):
        """
        Add foot pressure sensor feedback (Currently not implemented)

        :param floor: The floor object
        """

        foot_pressure_values = self.get_foot_pressure_sensors(floor)

        motor_forces = deepcopy(self.max_forces)
        if foot_pressure_values is None:
            return motor_forces

        # if (foot_pressure_values[0] and foot_pressure_values[1]) or (
        #         foot_pressure_values[2] and foot_pressure_values[3]):  # Right foot on the ground
        #     motor_forces[Joints.RIGHT_LEG_6] = 0.75
        # if (foot_pressure_values[4] and foot_pressure_values[5]) or (
        #         foot_pressure_values[6] and foot_pressure_values[7]):  # Right foot on the ground
        #     motor_forces[Joints.LEFT_LEG_6] = 0.75

        # Synchronise walking speed

        return motor_forces

    def publishAngles(self):
        """
        Publishes angles to ros
        """
