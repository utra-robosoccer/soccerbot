import matplotlib.pyplot as plt
import numpy as np
import scipy

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

        if np.round(np.linalg.norm([Xd, Yd, Zd]), 4) > (d3 + d4):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert np.round(np.linalg.norm([Xd, Yd, Zd]), 4)

        theta6 = -np.arctan2(Yd, Zd)
        tmp1 = Zd / np.cos(theta6)
        tmp2 = Xd
        D = np.clip((((((tmp1**2) + (tmp2**2)) - ((d3**2) + (d4**2))) / 2) / d3) / d4, -1, 1)  # TODO what the fuck is this
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

        if np.round(np.linalg.norm([Xd, Yd, Zd]), 4) > (d3 + d4):
            print("IK Position Unreachable: Desired Distance: " + str(np.linalg.norm([Xd, Yd, Zd])) + ", Limited Distance: " + str(d3 + d4))
        assert np.round(np.linalg.norm([Xd, Yd, Zd]), 4) <= (d3 + d4)

        theta6 = -np.arctan2(Yd, Zd)
        r_2 = Xd**2 + Yd**2 + Zd**2
        num = d3**2 + d4**2 - r_2
        denom = 2 * d3 * d4
        theta4 = np.arccos(np.clip(num / denom, -1, 1)) - np.pi  # TODO not sure why its neg

        assert theta4 < 4.6

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


# ik = IK()
# A = 0.089
# B = 0.0827
# H = 0.0 # 0.05 # up to 0.14 then they are equivelent but different angles
# x = np.linspace(-(A + B-H), A + B-H)
# xx = []
# zz = []
# for i in x:
#     # t = Transformation(position=[0.1835, -0.035, -0.17])  # , quaternion=(0.0, 0.0, 0.7, 0.7))
#     z = -np.sqrt((A + B-H) ** 2 - i ** 2) - 0.156
#     t = Transformation(position=[i + 0.0135, -0.035, z])
#     # # TODO should add a limit above -0.156 also not the same
#     print(np.linalg.norm(t.position - [0.0135, -0.035, -0.156]), ik.DH[2, 0] + ik.DH[3, 0])
#     # if np.linalg.norm(t.position - [0.0135, -0.035, -0.156]) > ik.DH[2, 0] + ik.DH[3, 0]: # TODO fix out of bound
#     #     continue
#
#     zz.append(z)
#     xx.append(i + 0.0135)
#     print(i + 0.0135, z)
#     # TODO script to take user input for ik for testing
#     ik1 = ik.inverseKinematicsRightFoot(np.copy(t))
#     ik2 = ik.inverseKinematicsRightFoot2(np.copy(t))
#     print("ik1: ", ik1)
#     print("ik2: ", ik2)
#     assert np.isclose(ik1, ik2).all()
#     # [-0.0, 0.0, 2.031230955889592, -1.5673589601356506, -0.4638719957539408, 0.0]
# plt.scatter(xx, zz)
# plt.show()

# ik = IK()
# A = 0.089
# B = 0.0827
# H = 0.0  # 0 to 0.09
# y = np.linspace(-(A + B - H), A + B - H)
# yy = []
# zz = []
# for i in y:
#     # t = Transformation(position=[0.1835, -0.035, -0.17])  # , quaternion=(0.0, 0.0, 0.7, 0.7))
#     z = -np.sqrt((A + B - H) ** 2 - i ** 2) - 0.156
#     t = Transformation(position=[0.0135, i - 0.035, z])
#     # # TODO should add a limit above -0.156 also not the same
#     print(np.linalg.norm(t.position - [0.0135, -0.035, -0.156]), ik.DH[2, 0] + ik.DH[3, 0])
#     try:
#         ik1 = ik.inverseKinematicsRightFoot(np.copy(t))
#         ik2 = ik.inverseKinematicsRightFoot2(np.copy(t))
#     except Exception as e:
#         print(e)
#         continue
#
#     zz.append(z)
#     yy.append(i - 0.035)
#     print(i - 0.035, z)
#     # TODO script to take user input for ik for testing
#     print("ik1: ", ik1)
#     print("ik2: ", ik2)
#     print(np.isclose(ik1, ik2, rtol=1.0e-5, atol=1.0e-7))
#     assert np.isclose(ik1, ik2, rtol=1.0e-5, atol=1.0e-7).all()
#
# plt.scatter(yy, zz)
# plt.show()
# TODO make them seperate functions for each axis out in pybullet
# TODO split the IK between 4,5,6 and 1,2,3

ik = IK()
A = 0.089
B = 0.0827
z = np.linspace(-(A + B), -0.04)
yy = []
zz = []
for i in z:
    # t = Transformation(position=[0.1835, -0.035, -0.17])  # , quaternion=(0.0, 0.0, 0.7, 0.7))
    t = Transformation(position=[0.0135, -0.035, i - 0.156])
    # # TODO should add a limit above -0.156 also not the same
    print(np.linalg.norm(t.position - [0.0135, -0.035, -0.156]), ik.DH[2, 0] + ik.DH[3, 0])
    try:
        ik1 = ik.inverseKinematicsRightFoot(np.copy(t))
        ik2 = ik.inverseKinematicsRightFoot2(np.copy(t))
    except Exception as e:
        print(e)
        continue

    zz.append(i - 0.156)
    yy.append(-0.035)
    print(-0.035, i - 0.156)
    # TODO script to take user input for ik for testing
    print("ik1: ", ik1)
    print("ik2: ", ik2)
    print(np.isclose(ik1, ik2, rtol=1.0e-5, atol=1.0e-7))
    # assert np.isclose(ik1, ik2, rtol=1.0e-5, atol=1.0e-7).all()

plt.scatter(yy, zz)
plt.show()
# TODO now put into pybullet and play it
