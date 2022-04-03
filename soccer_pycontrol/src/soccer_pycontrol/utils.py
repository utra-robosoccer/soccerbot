import numpy as np

def wrapTo2Pi(num: float) -> float:
    rem = num % (2 * np.pi)
    return rem

def wrapToPi(num: float) -> float:
    rem = (num + np.pi) % (2 * np.pi) - np.pi
    return rem
