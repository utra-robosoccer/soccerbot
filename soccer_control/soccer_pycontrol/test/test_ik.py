import os
import unittest

import numpy as np
from matplotlib import pyplot as plt
from soccer_pycontrol.soccerbot.inverse_kinematics import InverseKinematics

os.environ["ROS_NAMESPACE"] = "/robot1"

PLOT = True


class TestIK(unittest.TestCase):
    def test_x_sweep(self):
        """
        Case 1: Standard case
        :return: None
        """
        ik = InverseKinematics()
        thetas, x, z = ik.x_sweep()
        # np.save('ik_data''/x_sweep', thetas)

        assert np.allclose(thetas, np.load("ik_data" "/x_sweep.npy"))
        if PLOT:
            plt.scatter(x, z)
            plt.show()

    def test_y_sweep(self):
        """
        Case 1: Standard case
        :return: None
        """
        ik = InverseKinematics()
        thetas, y, z = ik.y_sweep()
        # np.save('ik_data''/y_sweep', thetas)

        assert np.allclose(thetas, np.load("ik_data" "/y_sweep.npy"))
        if PLOT:
            plt.scatter(y, z)
            plt.show()

    def test_z_sweep(self):
        """
        Case 1: Standard case
        :return: None
        """
        ik = InverseKinematics()
        thetas, y, z = ik.z_sweep()
        # np.save('ik_data''/z_sweep', thetas)

        assert np.allclose(thetas, np.load("ik_data" "/z_sweep.npy"))
        if PLOT:
            plt.scatter(y, z)
            plt.show()
