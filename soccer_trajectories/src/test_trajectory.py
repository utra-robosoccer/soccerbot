import os
import sys
from importlib import reload
from os.path import exists
from types import ModuleType

import numpy as np
import pybullet as pb
import pytest
import yaml

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

from soccer_common.transformation import Transformation

real_robot = False
run_in_ros = True
display = False
TEST_TIMEOUT = 60

if run_in_ros:
    import rospy

    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_pycontrol'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_trajectories'")

else:
    sys.modules["rospy"] = MagicMock()
    sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
    import rospy

    rospy.Time = MagicMock()
    joint_state = MagicMock()
    joint_state.position = [0.0] * 18
    rospy.wait_for_message = MagicMock(return_value=joint_state)
    rospy.loginfo_throttle = lambda a, b: None

robot_model = "bez1"

import soccer_trajectories
from soccer_msgs.msg import FixedTrajectoryCommand
from soccer_trajectories import SoccerTrajectoryClass


class TestTrajectory:
    robot_models = ["bez1"]  # "bez3"]

    @staticmethod
    def reset_attributes():
        for i in range(2):
            for attribute_name in dir(soccer_trajectories):
                if attribute_name == __name__.replace(__package__ + ".", ""):
                    continue
                attribute = getattr(soccer_trajectories, attribute_name)
                if type(attribute) is ModuleType:
                    reload(attribute)

    @staticmethod
    @pytest.fixture(params=robot_models)
    def trajectory(request):
        global robot_model
        robot_model = request.param

        if run_in_ros:
            c = SoccerTrajectoryClass()
        else:
            TestTrajectory.reset_attributes()
            c = SoccerTrajectoryClass()  # need a version for pybullet
        yield c
        del c

    @pytest.mark.parametrize("trajectory", ["bez1"], indirect=True)
    def test_getupfront(self, trajectory: SoccerTrajectoryClass):
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = "getupfront"
        msg.mirror = False
        traj_success = trajectory.run_trajectory(command=msg)
        assert traj_success

    @pytest.mark.parametrize("trajectory", ["bez1"], indirect=True)
    def test_getupback(self, trajectory: SoccerTrajectoryClass):
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = "getupback"
        msg.mirror = False
        traj_success = trajectory.run_trajectory(command=msg)
        assert traj_success

    @pytest.mark.parametrize("trajectory", ["bez1"], indirect=True)
    def test_getupside(self, trajectory: SoccerTrajectoryClass):
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = "getupside"
        msg.mirror = False
        traj_success = trajectory.run_trajectory(command=msg)
        assert traj_success

    @pytest.mark.parametrize("trajectory", ["bez1"], indirect=True)
    def test_rightkick(self, trajectory: SoccerTrajectoryClass):
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = "rightkick"
        msg.mirror = False
        traj_success = trajectory.run_trajectory(command=msg)
        assert traj_success

    @pytest.mark.parametrize("trajectory", ["bez1"], indirect=True)
    def test_leftkick(self, trajectory: SoccerTrajectoryClass):
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = "rightkick"
        msg.mirror = True
        traj_success = trajectory.run_trajectory(command=msg)
        assert traj_success
