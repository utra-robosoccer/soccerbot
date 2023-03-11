import numpy as np


def wrapTo2Pi(num: float) -> float:
    """
    Wraps a angle to 2 pi, etc -5pi -> -pi
    :param num: Angle in radians
    """

    rem = num % (2 * np.pi)
    return rem


def wrapToPi(num: float) -> float:
    """
    Wraps a angle to pi, etc -3pi -> -pi
    :param num: Angle in radians
    """
    rem = (num + np.pi) % (2 * np.pi) - np.pi
    return rem


def trimToPi(num: float) -> float:
    """
    Limit a angle to be within -pi and pi
    :param num: angle in floats
    """

    return max(min(num, np.pi), -np.pi)
