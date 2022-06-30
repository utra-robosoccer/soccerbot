import os
import sys
from importlib import reload
from os.path import exists
from types import ModuleType

import pytest
import yaml

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

display = True
TEST_TIMEOUT = 60

sys.modules["rospy"] = MagicMock()
sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
import rospy

rospy.Time = MagicMock()
joint_state = MagicMock()
joint_state.position = [0.0] * 18
rospy.wait_for_message = MagicMock(return_value=joint_state)
rospy.loginfo_throttle = lambda a, b: None


def f(a, b, robot_model="bez1"):
    a = a.lstrip("~")
    if a == "robot_model":
        return robot_model

    config_path = f"../../config/{robot_model}_sim.yaml"
    if not exists(config_path):
        return b

    with open(config_path, "r") as g:

        y = yaml.safe_load(g)
        for c in a.split("/"):
            if y is None or c not in y:
                return b
            y = y[c]
        return y


class TestWalking:
    def test_obtain_calibration_bez1(self):
        global robot_model
        robot_model = "bez1"
        rospy.get_param = f
        from soccer_pycontrol.calibration import Calibration

        c = Calibration()
        c.obtain_calibration()
