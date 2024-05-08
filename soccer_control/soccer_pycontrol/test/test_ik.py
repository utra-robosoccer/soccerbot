import os
import unittest

import numpy as np
from matplotlib import pyplot as plt
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot.ik_actions import IKActions
from soccer_pycontrol.soccerbot.ik_data import IKData
from soccer_pycontrol.soccerbot.ik_equation import IKEquation

from soccer_common import Transformation

os.environ["ROS_NAMESPACE"] = "/robot1"

PLOT = True


class TestIK(unittest.TestCase):
    def test_x_sweep(self):
        """
        Case 1: Standard case
        :return: None
        """
        ik_data = IKData()
        ik_actions = IKActions(ik_data)
        thetas, x, z = ik_actions.x_sweep()
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
        ik_data = IKData()
        ik_actions = IKActions(ik_data)
        thetas, y, z = ik_actions.y_sweep()
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
        ik_data = IKData()
        ik_actions = IKActions(ik_data)
        thetas, y, z = ik_actions.z_sweep()
        # np.save('ik_data''/z_sweep', thetas)

        assert np.allclose(thetas, np.load("ik_data" "/z_sweep.npy"))
        if PLOT:
            plt.scatter(y, z)
            plt.show()

    def test_ready(self):
        ik_data = IKData()
        ik_actions = IKActions(ik_data)
        configuration = ik_actions.ready()
        # np.save('ik_data''/ready', configuration)

        assert np.allclose(configuration, np.load("ik_data" "/ready.npy"))
