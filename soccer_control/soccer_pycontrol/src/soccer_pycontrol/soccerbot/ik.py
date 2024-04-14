import time

import numpy as np
import scipy
from scipy.spatial.transform import Rotation as R

from soccer_common import Transformation


class IK:
    def __init__(self):
        self.DH = np.array(
            [
                [0, -np.pi / 2, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0.089, 0, 0, 0],
                [0.0827, 0, 0, 0],
                [0, np.pi / 2, 0, 0],
                [0, 0, 0, 0],
            ]
        )

    def inverseKinematicsRightFoot(self, transformation: Transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        # TODO add plots and verify calculations
        # TODO all IK is done from 0,0,0 assumption
        # TODO make the library independent from pybullet
        transformation[0:3, 3] = transformation[0:3, 3] - Transformation(position=[0.0135, -0.035, -0.156])[0:3, 3]  # self.torso_to_right_hip[0:3, 3]
        invconf = scipy.linalg.inv(transformation)  # TODO why
        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4)

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = (((((tmp1**2) + (tmp2**2)) - ((d3**2) + (d4**2))) / 2) / d3) / d4  # TODO what the fuck is this
        tmp3 = np.arctan2(D, -np.sqrt(1 - (D**2)))  # TODO how is this worse

        tmpX = tmp3 - (np.pi / 2)
        if tmpX < 0:
            tmpX = tmpX + (2 * np.pi)
        theta4 = -(np.unwrap([tmpX])[0])

        assert theta4 < 4.6

        alp = np.arctan2(tmp1, tmp2)
        beta = np.arctan2(-d3 * np.cos(tmp3), d4 + (d3 * np.sin(tmp3)))  # TODO why
        theta5 = np.pi / 2 - (alp - beta)

        H34 = Transformation(dh=[self.DH[3, 0], self.DH[3, 1], self.DH[3, 2], theta4])
        H45 = Transformation(dh=[self.DH[4, 0], self.DH[4, 1], self.DH[4, 2], theta5])
        H56 = Transformation(dh=[self.DH[5, 0], self.DH[5, 1], self.DH[5, 2], theta6])
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(H36))
        assert np.linalg.norm(H03[0:3, 3]) - d3 < 0.03

        angles = Transformation(rotation_matrix=scipy.linalg.inv(H03[0:3, 0:3])).orientation_euler

        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2
        theta3 = np.pi / 2 - angles[0]

        return [theta1, theta2, theta3, theta4, theta5, theta6]

    def inverseKinematicsRightFoot2(self, transformation: Transformation):
        """
        # Does the inverse kinematics calculation for the right foot

        :param transformation: The 3D transformation from the torso center to the foot center
        :return: 6x1 Motor angles for the right foot
        """
        transformation[0:3, 3] = transformation[0:3, 3] - Transformation(position=[0.0135, -0.035, -0.156])[0:3, 3]  # self.torso_to_right_hip[0:3, 3]
        invconf = scipy.linalg.inv(transformation)  # TODO why
        d3 = self.DH[2, 0]
        d4 = self.DH[3, 0]

        Xd = invconf[0, 3]
        Yd = invconf[1, 3]
        Zd = invconf[2, 3]

        if np.linalg.norm([Xd, Yd, Zd]) > (d3 + d4):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert np.linalg.norm([Xd, Yd, Zd]) <= (d3 + d4)

        theta6 = -np.arctan2(Yd, Zd)
        r_2 = Xd**2 + Yd**2 + Zd**2
        num = d3**2 + d4**2 - r_2
        denom = 2 * d3 * d4
        theta4 = np.arccos(num / denom) - np.pi  # TODO not sure why its neg

        num = d3 * np.sin(np.pi + theta4)
        denom = np.sqrt(r_2)
        alpha = np.arcsin(num / denom)
        theta5 = np.arctan2(Xd, np.sign(Zd) * np.sqrt(Yd**2 + Zd**2)) + alpha

        H34 = Transformation(dh=[d4, 0, 0, theta4])
        H45 = Transformation(dh=[0, np.pi / 2, 0, theta5])
        H56 = Transformation(dh=[0, 0, 0, theta6])
        H36 = np.matmul(H34, np.matmul(H45, H56))
        final_rotation = Transformation(euler=[0, np.pi / 2, np.pi])
        H03 = np.matmul(np.matmul(transformation, final_rotation), scipy.linalg.inv(H36))
        assert np.linalg.norm(H03[0:3, 3]) - d3 < 0.03  # TODO why
        angles = Transformation(rotation_matrix=scipy.linalg.inv(H03[0:3, 0:3])).orientation_euler

        theta1 = -angles[1]
        theta2 = angles[2] + np.pi / 2
        theta3 = np.pi / 2 - angles[0]
        return [theta1, theta2, theta3, theta4, theta5, theta6]


ik = IK()
t = Transformation(position=[0.0135, 0.06, -0.29289])  # , quaternion=(0.0, 0.0, 0.7, 0.7))
# print(t.rotation_matrix)
#  -0.3188203886880778, 0.16530755885403078, -0.6302562811565202]

# -6.060296883738752e-14, 0.6307211857938324, 0.15238157118365828, -0.31647168907303647, 0.16409011788943692, -2.20142051098441]
for i in range(1000):
    start = time.process_time_ns()
    print(ik.inverseKinematicsRightFoot(np.copy(t)))
    print((time.process_time_ns() - start) / 1e6)
    start = time.process_time_ns()
    print(ik.inverseKinematicsRightFoot2(np.copy(t)))
    print((time.process_time_ns() - start) / 1e6)


# def tmp():
#     print((time.process_time_ns() - start) / 1e6)
#     print(theta4,theta5,theta6)
#     start = time.process_time_ns()
#     r_2 = Xd ** 2 + Yd ** 2 + Zd ** 2
#     num = (d3 ** 2 + d4 ** 2 - r_2)
#     denom = (2 * d3 * d4)
#     theta4_2 = np.arccos(num / denom) - np.pi  # TODO not sure why its neg
#
#     num = d3 * np.sin(np.pi + theta4_2)
#     denom = np.sqrt(r_2)
#     alpha = np.arcsin(num / denom)
#     theta5_2 = np.arctan2(Xd, np.sign(Zd) * np.sqrt(Yd ** 2 + Zd ** 2)) + alpha
#     print((time.process_time_ns() - start) / 1e6)
#     print(theta4_2, theta5_2, theta6)
#     -1.2971196567286807 0.6763627196264181 0.0
#     [
#         -0.0,
#         -2.220446049250313e-16,
#         0.6207569371022629,
#         -1.2971196567286807,
#         0.6763627196264181,
#         0.0,
#     ]
