import copy
import numpy as np


class Ball:
    FRICTION_COEFF = 0.8

    def __init__(self, position):
        self.position = position
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0

    def get_position(self):
        return copy.deepcopy(self.position)

    def get_velocity(self):
        return copy.deepcopy(self.velocity)

    def is_moving(self):
        return np.linalg.norm(self.velocity) != 0