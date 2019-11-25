import numpy as np

def mcuToCtrlAngles(mcuAngles):
    """ Applies a linear transformation to the motor angles
        received from the embedded systems to convert them to
        the coordinate system used by the control systems
    """
    arr = np.zeros((18, 1), dtype=np.float)
    arr[:mcuAngles.shape[0], :mcuAngles.shape[1]] = mcuAngles

    # Additive transformation factor
    a = np.array([150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 60, 150, 240, 150, 150])
    a = a[:, np.newaxis]

    # Multiplicative transformation factor
    m = np.array([1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1])
    m = m[:, np.newaxis]
    m = m * 180.0 / np.pi

    return (arr - a) / m