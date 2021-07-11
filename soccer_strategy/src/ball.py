import numpy as np


class Ball:
    FRICTION_COEFF = 0.8

    def __init__(self, position):
        self.position = position
        self.position_timeout = True
        self.position_is_live_timeout = 0
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0

    def get_position(self):
        if self.position_is_live_timeout == 0 and self.position_timeout:
            return None
        return self.position

    def get_velocity(self):
        return self.velocity