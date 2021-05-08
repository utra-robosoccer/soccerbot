import numpy as np


class Ball:
    FRICTION_COEFF = 0.8

    def __init__(self, position):
        self.position = position
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0

    def get_position(self):
        return self.position

    def get_velocity(self):
        return self.velocity