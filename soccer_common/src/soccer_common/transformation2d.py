"""EXPERIMENTAL, USE AT YOUR OWN RISK!"""
import math

import numpy as np


class Transformation2D(np.ndarray):
    def __new__(cls, matrix=None, pos_theta=None, *args, **kwargs):
        cls = np.eye(3).view(cls)

        if matrix is not None:
            cls.matrix = matrix
        else:
            cls.pos_theta = pos_theta
        return cls

    @property
    def position(self) -> np.ndarray:
        # Position in form [x y]
        return np.array(self[0:2, 2])

    @position.setter
    def position(self, position: [float]):
        self[0:2, 2] = position

    @property
    def yaw(self) -> float:
        return math.atan2(self[1, 0], self[0, 0])

    @yaw.setter
    def yaw(self, yaw: float):
        self[0, 0] = math.cos(yaw)
        self[0, 1] = -math.sin(yaw)
        self[1, 0] = math.sin(yaw)
        self[1, 1] = math.cos(yaw)

    @property
    def matrix(self) -> np.ndarray:
        return np.array(self)

    @matrix.setter
    def matrix(self, matrix: np.array):
        self[0:3, 0:3] = matrix

    @property
    def pos_theta(self):
        # Field in form [x, y, yaw]
        return np.array([self.position[0], self.position[1], self.yaw])

    @pos_theta.setter
    def pos_theta(self, pos_theta: [float]):
        self.position = (pos_theta[0], pos_theta[1])
        self.yaw = pos_theta[2]
