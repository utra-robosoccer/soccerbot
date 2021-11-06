import numpy as np


class Ball:
    FRICTION = 1
    FRICTION_COEFF = 0.8

    def __init__(self, position, simulator='3D'):
        self.position = position
        if simulator == '3D':
            self.position_is_live_timeout = 0
        else:
            self.position_is_live_timeout = 1
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0

    def get_position(self):
        if self.position_is_live_timeout == 0:
            return None
        return self.position

    def get_velocity(self):
        return self.velocity

    def is_moving(self):
        return np.linalg.norm(self.velocity) < 0.1  # compensate for noise
