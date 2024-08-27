# from typing import List, Tuple
#
# import numpy as np
#
# from soccer_common import Transformation
#
#
# class IKActions:
#     """
#     Class for actions robot will do using ik
#     """
#
#     def __init__(self,  motor_names: List[str]):
#         self.motor_names = motor_names
#         self.leg_length = self.data.thigh_length + self.data.tibia_length
#         # self.ik = IKCalculation(self.data.thigh_length, self.data.tibia_length)
#
#     # TODO Should be in separate file ?
#     def calc_x_z(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
#         x = np.linspace(-(self.leg_length - h), self.leg_length - h)
#         z = -np.sqrt((self.leg_length - h) ** 2 - x**2) + self.data.torso_to_right_hip.position[2]
#
#         return x, z
#
#     def x_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
#         # subtract from z curve 0.05 # up to 0.14 then they are equivalent but different angles
#         x, z = self.calc_x_z(h)
#
#         x += self.data.torso_to_right_hip.position[0]
#         thetas = []
#
#         for idx, xd in enumerate(x):
#             t = Transformation(position=[xd, self.data.torso_to_right_hip.position[1], z[idx]])
#             # TODO should add a limit above -0.156 also not the same
#             # TODO script to take user input for ik for testing
#             thetas.append(self.get_right_leg_angles(np.copy(t)))
#
#         return np.array(thetas), x, z
#
#     def y_sweep(self, h: float = 0.0) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
#         # 0 to 0.09
#         y, z = self.calc_x_z(h)
#
#         y += self.data.torso_to_right_hip.position[1]
#         thetas = []
#
#         for idx, yd in enumerate(y):
#             # TODO should add a limit above -0.156 also not the same
#             # TODO script to take user input for ik for testing
#
#             t = Transformation(position=[self.data.torso_to_right_hip.position[0], yd, z[idx]])
#
#             thetas.append(self.get_right_leg_angles(np.copy(t)))
#
#         return np.array(thetas), y, z
#
#     def z_sweep(self):
#         z = np.linspace(-self.leg_length, -0.04) + self.data.torso_to_right_hip.position[2]
#         thetas = []
#         for i in z:
#             t = Transformation(position=[self.data.torso_to_right_hip.position[0], self.data.torso_to_right_hip.position[1], i])
#             # TODO should add a limit above -0.156 also not the same
#             thetas.append(self.get_right_leg_angles(np.copy(t)))
#
#             # TODO script to take user input for ik for testing
#
#         return np.array(thetas), np.ones_like(z) * self.data.torso_to_right_hip.position[1], z
