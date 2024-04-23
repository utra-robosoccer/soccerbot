from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import scipy

from soccer_common import Transformation


class InverseKinematics:
    def __init__(
        self,
        thigh_length: float = 0.089,
        tibia_length: float = 0.0827,
        torso_to_right_hip: Transformation = Transformation(position=[0.0135, -0.035, -0.156]),
        right_hip_to_left_hip: Transformation = Transformation(position=[0, -0.07, 0]),
    ):
        self.thigh_length = thigh_length
        self.tibia_length = tibia_length

        # TODO should be a position vector
        self.torso_to_right_hip = torso_to_right_hip
        self.right_hip_to_left_hip = right_hip_to_left_hip

        self.DH = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [self.thigh_length, 0, 0, 0],
                [self.tibia_length, 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )

    def ik_right_foot_geometry(self, transformation: Transformation):
        invconf = scipy.linalg.inv(transformation)  # TODO why

        rx = invconf[0, 3]
        ry = invconf[1, 3]
        rz = invconf[2, 3]

        if np.round(np.linalg.norm([rx, ry, rz]), 4) > (self.thigh_length + self.tibia_length):
            print(
                "InverseKinematics Position Unreachable: Desired Distance: "
                + str(np.linalg.norm([rx, ry, rz]))
                + ", Limited Distance: "
                + str(self.thigh_length + self.tibia_length)
            )
        assert np.round(np.linalg.norm([rx, ry, rz]), 4) <= (self.thigh_length + self.tibia_length)

        theta6 = -np.arctan2(ry, rz)

        r_2 = rx**2 + ry**2 + rz**2
        num = self.thigh_length**2 + self.tibia_length**2 - r_2
        denom = 2 * self.thigh_length * self.tibia_length
        theta4 = np.arccos(np.clip(num / denom, -1, 1)) - np.pi  # TODO not sure why its neg

        assert theta4 < 4.6

        num = self.thigh_length * np.sin(np.pi + theta4)
        denom = np.sqrt(r_2)
        alpha = np.arcsin(num / denom)
        theta5 = np.arctan2(rx, np.sign(rz) * np.sqrt(ry**2 + rz**2)) + alpha

        return theta4, theta5, theta6

    def ik_right_foot_rotation(self, transformation: Transformation, theta4: float, theta5: float, theta6: float):
        T34 = Transformation(dh=[self.DH[3, 0], self.DH[3, 1], self.DH[3, 2], theta4])
        T45 = Transformation(dh=[self.DH[4, 0], self.DH[4, 1], self.DH[4, 2], theta5])
        T56 = Transformation(dh=[self.DH[5, 0], self.DH[5, 1], self.DH[5, 2], theta6])
        T36 = np.matmul(T34, np.matmul(T45, T56))

        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        T03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(T36))

        assert np.linalg.norm(T03[0:3, 3]) - self.thigh_length < 0.03  # TODO why
        angles = Transformation(rotation_matrix=scipy.linalg.inv(T03[0:3, 0:3])).orientation_euler

        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2
        theta3 = np.pi / 2 - angles[0]
        return theta1, theta2, theta3

    def ik_right_foot(self, transformation: Transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        # TODO make this notation simpler
        transformation[0:3, 3] = transformation[0:3, 3] - self.torso_to_right_hip[0:3, 3]

        theta4, theta5, theta6 = self.ik_right_foot_geometry(transformation)

        theta1, theta2, theta3 = self.ik_right_foot_rotation(transformation, theta4, theta5, theta6)

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def ik_left_foot(self, transformation: Transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: Motor angles for the left foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] + self.right_hip_to_left_hip[0:3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.ik_right_foot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]

    def calc_x_z(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
        x = np.linspace(-(self.thigh_length + self.tibia_length - h), self.thigh_length + self.tibia_length - h)
        z = -np.sqrt((self.thigh_length + self.tibia_length - h) ** 2 - x**2) + self.torso_to_right_hip.position[2]

        return x, z

    def x_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # subtract from z curve 0.05 # up to 0.14 then they are equivalent but different angles
        x, z = self.calc_x_z(h)

        x += self.torso_to_right_hip.position[0]
        thetas = []

        for idx, xd in enumerate(x):
            t = Transformation(position=[xd, self.torso_to_right_hip.position[1], z[idx]])
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing
            thetas.append(self.ik_right_foot(np.copy(t)))

        return np.array(thetas), x, z

    def y_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # 0 to 0.09
        y, z = self.calc_x_z(h)

        y += self.torso_to_right_hip.position[1]
        thetas = []

        for idx, yd in enumerate(y):
            # TODO should add a limit above -0.156 also not the same
            # TODO script to take user input for ik for testing

            t = Transformation(position=[self.torso_to_right_hip.position[0], yd, z[idx]])

            thetas.append(self.ik_right_foot(np.copy(t)))

        return np.array(thetas), y, z

    def z_sweep(self):
        z = np.linspace(-(self.thigh_length + self.tibia_length), -0.04) + self.torso_to_right_hip.position[2]
        thetas = []
        for i in z:
            t = Transformation(position=[self.torso_to_right_hip.position[0], self.torso_to_right_hip.position[1], i])
            # TODO should add a limit above -0.156 also not the same
            thetas.append(self.ik_right_foot(np.copy(t)))

            # TODO script to take user input for ik for testing

        return np.array(thetas), np.ones_like(z) * self.torso_to_right_hip.position[1], z


# TODO make them seperate functions for each axis out in pybullet
if __name__ == "__main__":
    ik = InverseKinematics()

# TODO now put into pybullet and play it
