import numpy as np
import rospy


class Ball:
    FRICTION = 1
    FRICTION_COEFF = 0.8

    def __init__(self, position=np.array([0, 0])):
        self.position = position
        self.last_observed_time_stamp = rospy.Time.now()
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0

    def is_known(self):
        if self.position is None:
            return False
        return len(self.position) > 0

    def is_moving(self):
        return np.linalg.norm(self.velocity) < 0.1  # compensate for noise
