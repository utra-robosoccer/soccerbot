import os

import pytest

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

real_robot = False

display = False
TEST_TIMEOUT = 60

os.system(
    "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
)

robot_model = "bez1"
if real_robot:
    from soccer_msgs.msg import FixedTrajectoryCommand
else:
    from soccer_common.mock_ros import mock_ros

    file_path = os.path.dirname(os.path.abspath(__file__))

    config_path = f"{file_path}/../../../{robot_model}_description/config/motor_mapping.yaml"

    from unittest.mock import MagicMock

    mock_ros(robot_model=robot_model, real_robot=real_robot, param_path=config_path)
    FixedTrajectoryCommand = MagicMock()

from soccer_trajectories.soccer_trajectories import SoccerTrajectoryClass


class TestTrajectory:
    robot_models = ["bez1"]

    @staticmethod
    @pytest.fixture(params=robot_models)
    def trajectory(request):
        global robot_model
        robot_model = request.param

        c = SoccerTrajectoryClass()

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
