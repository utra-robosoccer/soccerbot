"""
Environment
"""

import numpy as np
import pybullet as p
import matplotlib as plt
from time import sleep


class Ramp:
    """
    creates a ramp using a plane at location path with basePosition = position and baseOrientation = orientation
    where orientation is in yaw, pitch, roll
    """

    def __init__(self, path, position, orientation):
        self.orientation = orientation
        self.position = position
        self.path = path
        self.plane = p.loadURDF(self.path, basePosition=self.position,
                                baseOrientation=p.getQuaternionFromEuler(self.orientation))

    def setOrientation(self, orientation):
        p.removeBody(self.plane)
        self.__init__(self.path, self.position, orientation)

    def setPosition(self, position):
        p.removeBody(self.plane)
        self.__init__(self.path, position, self.orientation)