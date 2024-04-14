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

        start = time.process_time_ns()
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

        H01 = Transformation(dh=[0, np.pi / 2, 0, theta1])
        H12 = Transformation(dh=[0, np.pi / 2, 0, theta2 + np.pi / 2])
        H23 = Transformation(dh=[d3, 0, 0, theta3])
        H03_1 = np.matmul(H01, np.matmul(H12, H23))

        H34 = Transformation(dh=[d4, 0, 0, theta4])
        H45 = Transformation(dh=[0, -np.pi / 2, 0, theta5])
        H56 = Transformation(dh=[0, 0, 0, theta6])
        H36_2 = np.matmul(H34, np.matmul(H45, H56))
        H03_2 = np.matmul(transformation, scipy.linalg.inv(H36_2))
        # H03_2 = np.matmul(transformation, scipy.linalg.inv(H36))
        # H03_2 = np.matmul(np.matmul(transformation, Transformation(euler=[0, np.pi / 2, 0])), scipy.linalg.inv(H36_2))
        R12 = -np.sin(theta2) * np.cos(theta3)  # H03_2[0,1]
        R22 = np.cos(theta2) * np.cos(theta3)  # H03_2[1,1]
        R32 = np.sin(theta3)  # H03_2[2,1]
        R31 = -np.cos(theta3) * np.sin(theta4)  # H03_2[2,0]
        R33 = np.cos(theta3) * np.cos(theta4)  # H03_2[2,2]
        R12 = H03_2[0, 1]
        R22 = H03_2[1, 1]
        R32 = H03_2[2, 1]
        # R31 =  H03_2[2,0]
        # R33 =  H03_2[2,2]
        # t1 = np.arctan2(-R12, R22)
        # t2 = np.arctan2(R32, -R12*np.sin(theta2) +R22*np.cos(theta2))
        # t3 = np.arctan2(-R31, R33)
        # # print(H03_2)
        # print(np.isclose(H03_2, H03_1))
        # print(np.isclose(H03_2, H03))
        # angles_2 = Transformation(rotation_matrix=scipy.linalg.inv(H03_2[0:3, 0:3])).orientation_euler
        # # H06 = np.matmul(H03_1,H36_2)
        # # print(H06[0:3, 0:3])
        # # print("here: ", np.matmul(H06, final_rotation)[0:3, 0:3])
        #
        # theta1_2 = -angles_2[2] - np.pi / 2
        # theta2_2 = -angles_2[1]
        # theta3_2 = -angles_2[0]
        r1 = R.from_euler("zxy", [theta1, theta2, theta3])
        r2 = R.from_euler("yx", [theta4 + theta5, theta6])
        r3 = r1 * r2
        # print(r3.as_matrix())
        print(transformation[0:3, 0:3])

        # -6.060296883738752e-14, 0.6307211857938324, 0.15238157118365828,
        # print(np.matmul(transformation, final_rotation)[0:3, 0:3])
        # print(theta1, theta2, theta3)
        x1 = transformation[2, 0] * (np.sin(theta4) * np.sin(theta5) * np.sin(theta6) - np.cos(theta4) * np.cos(theta5) * np.cos(theta6))
        x2 = transformation[2, 1] * (-np.sin(theta4) * np.sin(theta5) * np.sin(theta6) + np.cos(theta4) * np.cos(theta5) * np.sin(theta6))
        x3 = transformation[2, 2] * (-np.sin(theta5) * np.cos(theta5) - np.sin(theta4) * np.cos(theta5))

        # (-transformation[1,0] * np.sin(theta6) - transformation[1,1] * np.cos(theta6))/(np.sin(theta2 + np.pi/2))
        t2 = np.arccos(transformation[2, 0] * np.sin(theta6) + transformation[2, 1] * np.cos(theta6)) - np.pi / 2
        t1 = np.arcsin((-transformation[1, 0] * np.sin(theta6) - transformation[1, 1] * np.cos(theta6)) / (np.sin(t2 + np.pi / 2)))
        t3 = np.arccos((x1 + x2 + x3) / (np.sin(t2 + np.pi / 2)))
        print("hi")
        # -2.220446049250313e-16 0.6309126945505448 0.15353047774239625
        #
        # return [theta1, theta2, theta3, theta4, theta5, theta6]


ik = IK()
t = Transformation(position=[0.0135, -0.135, -0.29289])  # , quaternion=(0.7, 0.0, 0.0, 0.7))
# print(t.rotation_matrix)
#  -0.3188203886880778, 0.16530755885403078, -0.6302562811565202]

# -6.060296883738752e-14, 0.6307211857938324, 0.15238157118365828, -0.31647168907303647, 0.16409011788943692, -2.20142051098441]
start = time.process_time_ns()
ik.inverseKinematicsRightFoot(np.copy(t))
# print((time.process_time_ns() - start) / 1e6)


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


# H34 = Transformation(dh=[d4, 0,0, theta4])
# H45 = Transformation(dh=[0, -np.pi /2, 0, theta5])
# H56 = Transformation(dh=[0,0,0, theta6])
# H362 = np.matmul(H34, np.matmul(H45, H56))
# print(H36, H362)
# H03_2 = np.matmul(transformation, scipy.linalg.inv(H362))
#
# angles_2 = Transformation(rotation_matrix=H03_2[0:3, 0:3]).orientation_euler
#
# theta1_2 = -angles_2[2] + np.pi / 2
# theta2_2 = angles_2[0]
# theta3_2 = -angles_2[1]
# print(theta1_2, theta2_2, theta3_2)
