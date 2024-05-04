import os
import unittest

import numpy as np
from matplotlib import pyplot as plt
from soccer_pycontrol.links import Links
from soccer_pycontrol.soccerbot.ik_actions import IKActions
from soccer_pycontrol.soccerbot.ik_data import IKData
from soccer_pycontrol.soccerbot.inverse_kinematics import InverseKinematics

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
        ik = InverseKinematics(ik_data)
        ik_actions = IKActions(ik)
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
        ik = InverseKinematics(ik_data)
        ik_actions = IKActions(ik)
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
        ik = InverseKinematics(ik_data)
        ik_actions = IKActions(ik)
        thetas, y, z = ik_actions.z_sweep()
        # np.save('ik_data''/z_sweep', thetas)

        assert np.allclose(thetas, np.load("ik_data" "/z_sweep.npy"))
        if PLOT:
            plt.scatter(y, z)
            plt.show()

    def test_thetas(self):
        ik_data = IKData()
        ik = InverseKinematics(ik_data)
        ik_actions = IKActions(ik)
        configuration = [0.0] * 18
        configuration_2 = [0.0] * 18

        # right leg
        # TODO revisit naming
        thetas = ik.ik_right_foot(Transformation(position=[-0.085, -0.035, -0.29289]))

        configuration_2[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]

        previous_configuration = configuration
        for r in np.arange(0, 1.00, 0.040):
            configuration[0:18] = (
                np.array(np.array(configuration_2[0:18]) - np.array(previous_configuration[0:18])) * r + np.array(previous_configuration[0:18])
            ).tolist()

            print(configuration)
