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

        #: Ready Pose angle for arm 1
        self.arm_0_center = rospy.get_param("arm_0_center", -0.45)

        #: Ready Pose angle for arm 2
        self.arm_1_center = rospy.get_param("arm_0_center", np.pi * 0.8)

        rospy.loginfo("List of joints for loaded URDF")
        rospy.loginfo([pb.getJointInfo(self.body, i)[1] for i in range(pb.getNumJoints(self.body))])

        self.configuration = [0.0] * len(Joints)  #: The 18x1 float array motor angle configuration for the robot's 18 motors
        self.configuration_offset = [0.0] * len(Joints)  #: The offset for the 18x1 motor angle configurations
        self.max_forces = []

        for i in range(0, 18):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or rospy.get_param("max_force", 6))

        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=self.get_angles(),
            forces=self.max_forces,
        )

        # For head rotation
        self.head_step = 0.0

        self.get_ready_rate = rospy.get_param("get_ready_rate", 0.02)

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All 18 angles of the robot in an array formation
        """
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def ready(self) -> None:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """

        # hands
        configuration = [0.0] * len(Joints)
        configuration[Joints.RIGHT_ARM_1] = self.arm_0_center
        configuration[Joints.LEFT_ARM_1] = self.arm_0_center
        configuration[Joints.RIGHT_ARM_2] = self.arm_1_center
        configuration[Joints.LEFT_ARM_2] = self.arm_1_center

        # right leg
        thetas = self.inverseKinematicsRightFoot(np.copy(self.right_foot_init_position))
        configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        # left leg
        thetas = self.inverseKinematicsLeftFoot(np.copy(self.left_foot_init_position))
        configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]

        # head
        configuration[Joints.HEAD_1] = self.configuration[Joints.HEAD_1]
        configuration[Joints.HEAD_2] = self.configuration[Joints.HEAD_2]

        # Slowly ease into the ready position
        previous_configuration = self.configuration
        try:
            # TODO Remove ros dependency
            joint_state = rospy.wait_for_message("joint_states", JointState, timeout=3)
            indexes = [joint_state.name.index(motor_name) for motor_name in self.motor_names]
            previous_configuration = [joint_state.position[i] for i in indexes]
        except (ROSException, AttributeError) as ex:
            rospy.logerr(ex)
        except ValueError as ex:
            print(ex)
            rospy.logerr("Not all joint states are reported, cable disconnect?")
            rospy.logerr("Joint States")
            rospy.logerr(joint_state)
            rospy.logerr("Motor Names")
            print(self.motor_names)
            previous_configuration = [0] * len(Joints)

        for r in np.arange(0, 1.00, 0.040):
            rospy.loginfo_throttle(1, "Going into ready position")
            self.configuration[0:18] = (
                np.array(np.array(configuration[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
            ).tolist()
            self.publishAngles()
            if pb.isConnected():
                pb.setJointMotorControlArray(
                    bodyIndex=self.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 18, 1)),
                    targetPositions=self.get_angles(),
                    forces=self.max_forces,
                )
                pb.stepSimulation()
            else:
                rospy.sleep(self.get_ready_rate)

        self.configuration_offset = [0] * len(Joints)

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
