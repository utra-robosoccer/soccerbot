import math

import numpy as np


def heading_error(theta_desired: float, theta_current: float) -> float:
    """
    Calculates the position and orientation error between a current pose and a desired pose.

    :return: A tuple of (position_error, desired_yaw, heading_error) representing the errors.
    """

    head_error = theta_desired - theta_current

    # Normalize the heading error to be between -pi and pi.
    heading_error_norm = math.atan2(math.sin(head_error), math.cos(head_error))

    return heading_error_norm


def desired_yaw(self) -> float:
    # Calculate the desired yaw (angle of rotation) using the arctan2 function.
    numerator = self.goal_loc[1] - self.curr_loc[1]
    denominator = self.goal_loc[0] - self.curr_loc[0]
    return float(np.arctan2(numerator, denominator))


def position_error(goal_loc: np.ndarray, curr_loc: np.ndarray = (0, 0)):
    return float(np.linalg.norm(goal_loc - curr_loc))


def find_new_vel(goal_loc: list, curr_loc: list = (0, 0), max_vel: float = 0.1):
    x_error = abs(goal_loc[0] - curr_loc[0])
    y_error = abs(goal_loc[1] - curr_loc[1])

    if x_error > y_error:
        dx = max_vel
        dy = dx * (y_error / x_error)
    elif x_error < y_error:
        dy = max_vel
        dx = dy * (x_error / y_error)
    else:
        dx = max_vel
        dy = max_vel

    return math.copysign(dx, goal_loc[0]), math.copysign(dy, goal_loc[1])
