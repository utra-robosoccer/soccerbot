import numpy as np
import unittest


def getCtrlToMcuAngleMap():
    """ Converts an angle vector received from the control system into the angle
        vector used by the embedded systems. This involves a shuffling of
        positions and changing of signs

        m[embeddedID, ctrlID] = 1 to create a mapping
    """
    m = np.zeros((18, 18))

    # Right leg (Jason's Left)
    m[0, 11] = -1
    m[1, 10] = 1
    m[2, 9] = 1
    m[3, 8] = 1
    m[4, 7] = -1
    m[5, 6] = 1

    # Left leg (Jason's Right)
    m[6, 0] = 1
    m[7, 1] = -1
    m[8, 2] = 1
    m[9, 3] = 1
    m[10, 4] = 1
    m[11, 5] = -1

    #  Right arm
    m[12, 12] = 1
    m[13, 13] = 1

    # Left arm
    m[14, 14] = 1
    m[15, 15] = 1

    # Head
    m[16, 16] = 1
    m[17, 17] = 1

    return m


def ctrlToMcuAngles(ctrlAngles):
    """ Applies a linear transformation to the motor angles
        received from the control system to convert them to
        the coordinate system used by the motors
    """
    arr = np.zeros((18, 1), dtype=np.float)
    arr[:ctrlAngles.shape[0], :ctrlAngles.shape[1]] = ctrlAngles

    # Multiplicative transformation factor
    m = np.array([1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1])
    m = m[:, np.newaxis]
    m = m * 180.0 / np.pi

    # Additive transformation factor
    a = np.array([150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 60, 150, 240, 150, 150])
    a = a[:, np.newaxis]

    return (m * arr) + a


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


class TestCtrlMcuAngleMap(unittest.TestCase):
    def test_left_leg(self):
        m = getCtrlToMcuAngleMap()
        tv = np.zeros((18,))
        tv[0] = 6
        tv[1] = 5
        tv[2] = 4
        tv[3] = 3
        tv[4] = 2
        tv[5] = 1

        golden = np.zeros((18,))
        golden[11] = -tv[5]
        golden[10] = tv[4]
        golden[9] = tv[3]
        golden[8] = tv[2]
        golden[7] = -tv[1]
        golden[6] = tv[0]

        result = m.dot(tv)
        np.testing.assert_array_equal(result, golden)

    def test_right_leg(self):
        m = getCtrlToMcuAngleMap()
        tv = np.zeros((18,))
        tv[6] = 6
        tv[7] = 5
        tv[8] = 4
        tv[9] = 3
        tv[10] = 2
        tv[11] = 1

        golden = np.zeros((18,))
        golden[0] = -tv[11]
        golden[1] = tv[10]
        golden[2] = tv[9]
        golden[3] = tv[8]
        golden[4] = -tv[7]
        golden[5] = -tv[6]

        result = m.dot(tv)
        np.testing.assert_array_equal(result, golden)


if __name__ == "__main__":
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCtrlMcuAngleMap)
    unittest.TextTestRunner(verbosity=2).run(suite)