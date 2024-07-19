import numpy as np
import scipy

from soccer_common import Transformation


class IKCalculation:
    """
    Main inverse kinematic calculations.
    """

    def __init__(self, thigh_length: float = 0.089, tibia_length: float = 0.0827):
        self.thigh_length = thigh_length
        self.tibia_length = tibia_length

        # r, alpha, d, theta
        self.leg_dh = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [self.thigh_length, 0, 0, 0],
                [self.tibia_length, 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )

    def right_foot_geometry(self, transformation: Transformation):
        leg_length = self.thigh_length + self.tibia_length
        invconf = scipy.linalg.inv(transformation)  # TODO why

        rx = invconf[0, 3]
        ry = invconf[1, 3]
        rz = invconf[2, 3]

        # TODO should add unit test to test this limit and more robust the function
        if np.round(np.linalg.norm([rx, ry, rz]), 4) > leg_length:
            # TODO could be a function that changes if ros is enabled
            print(
                "IKCalculation Position Unreachable: Desired Distance: "
                + str(np.linalg.norm([rx, ry, rz]))
                + ", Limited Distance: "
                + str(leg_length)
            )
        assert np.round(np.linalg.norm([rx, ry, rz]), 4) <= leg_length

        theta6 = -np.arctan2(ry, rz)

        r_2 = rx**2 + ry**2 + rz**2
        num = self.thigh_length**2 + self.tibia_length**2 - r_2
        denom = 2 * self.thigh_length * self.tibia_length
        theta4 = np.arccos(np.clip(num / denom, -1, 1)) - np.pi  # TODO not sure why its neg

        # TODO should add unit test to test this limit and more robust the function
        assert theta4 < 4.6

        num = self.thigh_length * np.sin(np.pi + theta4)
        denom = np.sqrt(r_2)
        alpha = np.arcsin(num / denom)
        theta5 = np.arctan2(rx, np.sign(rz) * np.sqrt(ry**2 + rz**2)) + alpha

        # offset hack fix
        # theta4 -= np.arctan(2.4/15)
        # theta5 += np.arctan(2.4/15) # off by 2mm

        return theta4, theta5, theta6

    def right_foot_rotation(self, transformation: Transformation, theta4: float, theta5: float, theta6: float):
        T34 = Transformation(dh=[self.leg_dh[3, 0], self.leg_dh[3, 1], self.leg_dh[3, 2], theta4])
        T45 = Transformation(dh=[self.leg_dh[4, 0], self.leg_dh[4, 1], self.leg_dh[4, 2], theta5])
        T56 = Transformation(dh=[self.leg_dh[5, 0], self.leg_dh[5, 1], self.leg_dh[5, 2], theta6])
        T36 = np.matmul(T34, np.matmul(T45, T56))

        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        T03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(T36))

        # TODO should add unit test to test this limit and more robust the function
        assert np.linalg.norm(T03[0:3, 3]) - self.thigh_length < 0.03  # TODO why
        angles = Transformation(rotation_matrix=scipy.linalg.inv(T03[0:3, 0:3])).orientation_euler

        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2
        theta3 = np.pi / 2 - angles[0]
        return theta1, theta2, theta3

    @staticmethod
    def head_geometry(transformation: Transformation) -> [float, float]:
        vec = transformation.position
        theta1 = np.arctan2(vec[1], vec[0])
        if abs(theta1) == np.pi / 2:
            theta2 = np.arctan2(vec[2], vec[1] / np.sin(theta1))
        else:
            theta2 = np.arctan2(vec[2], vec[0] / np.cos(theta1))

        return theta1, theta2
