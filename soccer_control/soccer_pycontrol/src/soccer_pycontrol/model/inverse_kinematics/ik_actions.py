from typing import Tuple

import numpy as np
from soccer_pycontrol.common.joints import Joints
from soccer_pycontrol.common.links import Links
from soccer_pycontrol.model.inverse_kinematics.ik_calculation import IKCalculation
from soccer_pycontrol.model.inverse_kinematics.kinematic_data import KinematicData

from soccer_common import Transformation


class IKActions:
    """
    Class for actions robot will do using ik
    """

    def __init__(self, data: KinematicData):
        self.data = data
        self.leg_length = self.data.thigh_length + self.data.tibia_length
        self.ik = IKCalculation(self.data.thigh_length, self.data.tibia_length)

    def get_right_leg_angles(self, transformation: Transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        # TODO make this notation simpler
        transformation[0:3, 3] = transformation[0:3, 3] - self.data.torso_to_right_hip[0:3, 3]

        theta4, theta5, theta6 = self.ik.right_foot_geometry(transformation)

        theta1, theta2, theta3 = self.ik.right_foot_rotation(transformation, theta4, theta5, theta6)

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def get_left_leg_angles(self, transformation: Transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: Motor angles for the left foot
        """
        # TODO should add unit test to test this limit and more robust the function
        transformation[0:3, 3] = transformation[0:3, 3] + self.data.right_hip_to_left_hip[0:3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.get_right_leg_angles(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def ready(self) -> np.ndarray:
        """
        Sets the robot's joint angles for the robot to standing pose.
        """
        # TODO dont like this
        # hands
        configuration = [0.0] * len(self.data.motor_names)
        # if len(self.data.motor_names) == 18:
        #     elbow_number = '_1'
        # else:
        #     elbow_number = '_2'

        # configuration[self.data.motor_names.index('right_arm_motor_0')] = self.data.arm_0_center
        # configuration[self.data.motor_names.index('left_arm_motor_0')] = self.data.arm_0_center
        # configuration[self.data.motor_names.index('right_arm_motor_2')] = 1.5
        configuration[self.data.motor_names.index("left_arm_motor_2")] = 1.5

        # right leg
        # TODO revisit naming
        # thetas = self.get_right_leg_angles(np.copy(self.data.right_foot_init_position))

        # # thetas = self.inverseKinematicsRightFoot(Transformation(position=[ -0.085, -0.035, -0.29289]))
        # configuration[self.data.motor_names.index('right_leg_motor_0'):self.data.motor_names.index(
        #     'right_leg_motor_5') + 1] = thetas[0:6]
        #
        # # left leg
        # thetas = self.get_left_leg_angles(np.copy(self.data.left_foot_init_position))
        # configuration[self.data.motor_names.index('left_leg_motor_0'):self.data.motor_names.index(
        #     'left_leg_motor_5') + 1] = thetas[0:6]
        return np.array(configuration)

    # TODO Should be in separate file ?
    def calc_x_z(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        x = np.linspace(-(self.leg_length - h), self.leg_length - h)
        z = -np.sqrt((self.leg_length - h) ** 2 - x**2) + self.data.torso_to_right_hip.position[2]

        return x, z

    def x_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # subtract from z curve 0.05 # up to 0.14 then they are equivalent but different angles
        x, z = self.calc_x_z(h)

        x += self.data.torso_to_right_hip.position[0]
        thetas = []

        for idx, xd in enumerate(x):
            t = Transformation(position=[xd, self.data.torso_to_right_hip.position[1], z[idx]])
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing
            thetas.append(self.get_right_leg_angles(np.copy(t)))

        return np.array(thetas), x, z

    def y_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # 0 to 0.09
        y, z = self.calc_x_z(h)

        y += self.data.torso_to_right_hip.position[1]
        thetas = []

        for idx, yd in enumerate(y):
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing

            t = Transformation(position=[self.data.torso_to_right_hip.position[0], yd, z[idx]])

            thetas.append(self.get_right_leg_angles(np.copy(t)))

        return np.array(thetas), y, z

    def z_sweep(self):
        z = np.linspace(-self.leg_length, -0.04) + self.data.torso_to_right_hip.position[2]
        thetas = []
        for i in z:
            t = Transformation(position=[self.data.torso_to_right_hip.position[0], self.data.torso_to_right_hip.position[1], i])
            # TODO should add a limit above -0.156 also not the same
            thetas.append(self.get_right_leg_angles(np.copy(t)))

            # TODO script to take user input for ik for testing

        return np.array(thetas), np.ones_like(z) * self.data.torso_to_right_hip.position[1], z

    def head_localizing_sweep(self):  # TODO will have to be split into a strategy pattern later
        angles = np.concatenate([np.linspace(-np.pi / 2, np.pi / 2, 100), np.linspace(np.pi / 2, -np.pi / 2, 100)])
        # TODO make more legit later
        angles2 = np.concatenate(
            [
                np.linspace(np.pi / 6, np.pi / 3, 50),
                np.linspace(np.pi / 3, np.pi / 6, 50),
                np.linspace(np.pi / 6, np.pi / 3, 50),
                np.linspace(np.pi / 3, np.pi / 6, 50),
            ]
        )
        thetas = []
        for idx, i in enumerate(angles):
            t = Transformation(position=[np.cos(i), np.sin(i), angles2[idx]])
            thetas.append(self.ik.head_geometry(t))

        return np.array(thetas)

    # def head_localizing_sweep(self): # TODO will have to be split into a strategy pattern later
    #     angles = np.concatenate([np.linspace(-np.pi / 2, np.pi / 2, 100), np.linspace(np.pi / 2, -np.pi / 2, 100)])
    #     # TODO make more legit later
    #     angles2 = np.concatenate(
    #         [
    #             np.linspace(np.pi / 6, np.pi / 3, 50),
    #             np.linspace(np.pi / 3, np.pi / 6, 50),
    #             np.linspace(np.pi / 6, np.pi / 3, 50),
    #             np.linspace(np.pi / 3, np.pi / 6, 50),
    #         ]
    #     )
    #     thetas = []
    #     for idx, i in enumerate(angles):
    #         t = Transformation(position=[np.cos(i), np.sin(i), angles2[idx]])
    #         thetas.append(self.ik.head_geometry(t))
    #
    #     return np.array(thetas)
