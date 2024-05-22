import numpy as np
import scipy
from soccer_pycontrol.soccerbot2.ik.ik_data import IKData

from soccer_common import Transformation


class IKEquation:
    def __init__(self, ik_data: IKData):
        self.ik_data = ik_data

        # r, alpha, d, theta
        self.DH = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [self.ik_data.thigh_length, 0, 0, 0],
                [self.ik_data.tibia_length, 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )

    def ik_right_foot_geometry(self, transformation: Transformation):
        invconf = scipy.linalg.inv(transformation)  # TODO why

        rx = invconf[0, 3]
        ry = invconf[1, 3]
        rz = invconf[2, 3]

        # TODO should add unit test to test this limit and more robust the function
        if np.round(np.linalg.norm([rx, ry, rz]), 4) > (self.ik_data.thigh_length + self.ik_data.tibia_length):
            print(
                "IKEquation Position Unreachable: Desired Distance: "
                + str(np.linalg.norm([rx, ry, rz]))
                + ", Limited Distance: "
                + str(self.ik_data.thigh_length + self.ik_data.tibia_length)
            )
        assert np.round(np.linalg.norm([rx, ry, rz]), 4) <= (self.ik_data.thigh_length + self.ik_data.tibia_length)

        theta6 = -np.arctan2(ry, rz)

        r_2 = rx**2 + ry**2 + rz**2
        num = self.ik_data.thigh_length**2 + self.ik_data.tibia_length**2 - r_2
        denom = 2 * self.ik_data.thigh_length * self.ik_data.tibia_length
        theta4 = np.arccos(np.clip(num / denom, -1, 1)) - np.pi  # TODO not sure why its neg

        # TODO should add unit test to test this limit and more robust the function
        assert theta4 < 4.6

        num = self.ik_data.thigh_length * np.sin(np.pi + theta4)
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

        # TODO should add unit test to test this limit and more robust the function
        assert np.linalg.norm(T03[0:3, 3]) - self.ik_data.thigh_length < 0.03  # TODO why
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
        transformation[0:3, 3] = transformation[0:3, 3] - self.ik_data.torso_to_right_hip[0:3, 3]

        theta4, theta5, theta6 = self.ik_right_foot_geometry(transformation)

        theta1, theta2, theta3 = self.ik_right_foot_rotation(transformation, theta4, theta5, theta6)

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def ik_left_foot(self, transformation: Transformation):
        """
        Inverse kinematic function for the left foot. Works due to symmetry between left and right foot.

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: Motor angles for the left foot
        """
        # TODO should add unit test to test this limit and more robust the function
        transformation[0:3, 3] = transformation[0:3, 3] + self.ik_data.right_hip_to_left_hip[0:3, 3]
        [theta1, theta2, theta3, theta4, theta5, theta6] = self.ik_right_foot(transformation)
        return [-theta1, -theta2, theta3, theta4, theta5, -theta6]
