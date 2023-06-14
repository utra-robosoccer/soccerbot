import os

os.environ["ROS_NAMESPACE"] = "/robot1"

import pytest
import rospy

from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import FixedTrajectoryCommand
from soccer_trajectories.soccer_trajectories import Trajectory, TrajectoryManager


class TestTrajectory:
    @pytest.mark.parametrize("robot_model", ["bez1", "bez1_sim"])
    @pytest.mark.parametrize("trajectory_name", ["getupfront", "getupback", "getupside", "rightkick"])
    def test_all_trajectories(self, robot_model: str, trajectory_name: str):
        rospy.init_node("test")

        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )
        robot_model = "bez1"
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_path = f"{file_path}/../../../{robot_model}_description/config/motor_mapping.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)

        if "DISPLAY" not in os.environ:
            Trajectory.RATE = 10000
        c = TrajectoryManager()
        rospy.init_node("test")
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = trajectory_name
        msg.mirror = True
        c.command_callback(command=msg)
        c.trajectory.run(real_time=False)
