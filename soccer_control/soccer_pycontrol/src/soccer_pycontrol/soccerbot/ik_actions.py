from typing import List, Tuple

import numpy as np
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot.ik_data import IKData
from soccer_pycontrol.soccerbot.inverse_kinematics import InverseKinematics

from soccer_common import Transformation


class IKActions:
    def __init__(self, ik_data: IKData):
        self.ik_data = ik_data
        self.ik = InverseKinematics(self.ik_data)

    def ready(self) -> np.ndarray:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """

        # hands
        configuration = [0.0] * len(Joints)
        configuration[Joints.RIGHT_ARM_1] = self.ik_data.arm_0_center
        configuration[Joints.LEFT_ARM_1] = self.ik_data.arm_0_center
        configuration[Joints.RIGHT_ARM_2] = self.ik_data.arm_1_center
        configuration[Joints.LEFT_ARM_2] = self.ik_data.arm_1_center

        # right leg
        # TODO revisit naming
        thetas = self.ik.ik_right_foot(np.copy(self.ik_data.right_foot_init_position))

        # thetas = self.inverseKinematicsRightFoot(Transformation(position=[ -0.085, -0.035, -0.29289]))
        configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        # left leg
        thetas = self.ik.ik_left_foot(np.copy(self.ik_data.left_foot_init_position))
        configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]

        return np.array(configuration)

    def calc_x_z(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        x = np.linspace(-(self.ik_data.thigh_length + self.ik_data.tibia_length - h), self.ik_data.thigh_length + self.ik_data.tibia_length - h)
        z = -np.sqrt((self.ik_data.thigh_length + self.ik_data.tibia_length - h) ** 2 - x**2) + self.ik_data.torso_to_right_hip.position[2]

        return x, z

    def x_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # subtract from z curve 0.05 # up to 0.14 then they are equivalent but different angles
        x, z = self.calc_x_z(h)

        x += self.ik_data.torso_to_right_hip.position[0]
        thetas = []

        for idx, xd in enumerate(x):
            t = Transformation(position=[xd, self.ik_data.torso_to_right_hip.position[1], z[idx]])
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing
            thetas.append(self.ik.ik_right_foot(np.copy(t)))

        return np.array(thetas), x, z

    def y_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # 0 to 0.09
        y, z = self.calc_x_z(h)

        y += self.ik_data.torso_to_right_hip.position[1]
        thetas = []

        for idx, yd in enumerate(y):
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing

            t = Transformation(position=[self.ik_data.torso_to_right_hip.position[0], yd, z[idx]])

            thetas.append(self.ik.ik_right_foot(np.copy(t)))

        return np.array(thetas), y, z

    def z_sweep(self):
        z = np.linspace(-(self.ik_data.thigh_length + self.ik_data.tibia_length), -0.04) + self.ik_data.torso_to_right_hip.position[2]
        thetas = []
        for i in z:
            t = Transformation(position=[self.ik_data.torso_to_right_hip.position[0], self.ik_data.torso_to_right_hip.position[1], i])
            # TODO should add a limit above -0.156 also not the same
            thetas.append(self.ik.ik_right_foot(np.copy(t)))

            # TODO script to take user input for ik for testing

        return np.array(thetas), np.ones_like(z) * self.ik_data.torso_to_right_hip.position[1], z
