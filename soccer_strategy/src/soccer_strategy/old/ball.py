import numpy as np
import rclpy


class Ball:
    """
    Contains dynamic information about the ball based on a robot's estimate
    """

    FRICTION = 1
    FRICTION_COEFF = 0.8

    def __init__(self, position=np.array([0, 0])):
        self.position: np.ndarray = position
        self.last_observed_time_stamp = self.Time(0)
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0
