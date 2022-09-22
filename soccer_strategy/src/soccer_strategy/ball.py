import numpy as np
import rospy


class Ball:
    """
    Contains dynamic information about the ball based on a robot's estimate
    """

    FRICTION = 1
    FRICTION_COEFF = 0.8

    def __init__(self, position=np.array([0, 0])):
        self.position = position
        self.last_observed_time_stamp = rospy.Time.now()
        self.velocity = np.array([0, 0])
        self.kick_timeout = 0
