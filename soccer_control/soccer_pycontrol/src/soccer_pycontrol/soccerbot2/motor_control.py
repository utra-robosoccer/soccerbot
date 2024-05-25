from typing import List

import numpy as np
import pybullet as pb
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links

from soccer_common.utils import wrapToPi


class MotorControl:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, body: pb.loadURDF):
        """
        Class for interacting with pybullet model

        """
        self.body = body

        self.motor_names = [pb.getJointInfo(self.body, i)[1].decode("utf-8") for i in range(18)]

        # Todo make it numpy and add getter and setters
        self.configuration = [0.0] * len(Joints)  #: The 18x1 float array motor angle configuration for the robot's 18 motors
        self.configuration_offset = [0.0] * len(Joints)  #: The offset for the 18x1 motor angle configurations

        self.max_forces = []

        for i in range(0, 18):
            self.max_forces.append(pb.getJointInfo(self.body, i)[10] or 6)  # rospy.get_param("max_force", 6))

        self.set_motor()

    def get_angles(self):
        """
        Function for getting all the angles, combines the configuration with the configuration offset

        :return: All 18 angles of the robot in an array formation
        """
        angles = [wrapToPi(a + b) for a, b in zip(self.configuration, self.configuration_offset)]
        return angles

    def set_motor(self) -> None:
        pb.setJointMotorControlArray(
            bodyIndex=self.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=self.get_angles(),
            forces=self.max_forces,
        )

    def set_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[0:18] = target_angles

    def set_right_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = target_angles

    def set_left_leg_target_angles(self, target_angles: np.ndarray) -> None:
        self.configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = target_angles

    def set_leg_joint_3_target_angle(self, target: float) -> None:
        self.configuration_offset[Joints.LEFT_LEG_3] += target
        self.configuration_offset[Joints.RIGHT_LEG_3] += target

    def set_leg_joint_5_target_angle(self, target: float) -> None:
        self.configuration_offset[Joints.LEFT_LEG_5] += target
        self.configuration_offset[Joints.RIGHT_LEG_5] += target
