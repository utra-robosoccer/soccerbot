import os
import time
import unittest
from unittest import TestCase

from sensor_msgs.msg import JointState

from soccer_trajectories.pybullet_setup import PybulletSetup
from soccer_trajectories.trajectory import Trajectory
from soccer_trajectories.trajectory_manager import TrajectoryManager
from soccer_trajectories.trajectory_manager_ros import TrajectoryManagerRos

os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

import pytest
import rospy

from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import FixedTrajectoryCommand


class TestTrajectory(TestCase):
    def setup(self) -> None:
        super().setUpClass()

    def test_trajectory(self):
        """
        Case 1: Standard case
        :return: None
        """
        traj = Trajectory(os.path.join(os.path.dirname(__file__), "../trajectories/bez1_sim/getupfront.csv"))
        #
        joint_state = JointState()
        joint_state.name = [
            "right_leg_motor_0",
            "left_leg_motor_0",
            "right_leg_motor_1",
            "left_leg_motor_1",
            "right_leg_motor_2",
            "left_leg_motor_2",
            "right_leg_motor_3",
            "left_leg_motor_3",
            "right_leg_motor_4",
            "left_leg_motor_4",
            "right_leg_motor_5",
            "left_leg_motor_5",
            "right_arm_motor_0",
            "left_arm_motor_0",
            "right_arm_motor_1",
            "left_arm_motor_1",
            "head_motor_0",
            "head_motor_1",
        ]
        joint_state.position = [0.0] * 18

        traj.read_trajectory(joint_state)

        self.assertEqual(
            traj.create_joint_states(traj.max_time).position,
            [0.0, 0.0, 0.0, 0.0, 0.564, 0.564, -1.176, -1.176, 0.613, 0.613, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    def test_pybullet(self):
        """
        Case 1: Standard case
        :return: None
        """
        p = PybulletSetup(robot_model="bez2")
        p.wait(1)
        p.close()
        assert True

    def test_trajectory_manager(self):
        """
        Case 1: Standard case
        :return: None
        """
        time.sleep(1)
        tm = TrajectoryManager(os.path.join(os.path.dirname(__file__), "../trajectories/bez1_sim/getupfront.csv"), "bez1")
        tm.run()
        assert True
        # TODO add more testing from pybullet so like the height will reach a threshold and it doesnt fall over for
        #  some time

    def run_real_trajectory(self, robot_model: str, trajectory_name: str, real_time: bool):
        # TODO clean up
        rospy.init_node("test")
        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy "
            "/robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )

        file_path = os.path.dirname(os.path.abspath(__file__))
        config_path = f"{file_path}/../../../{robot_model}_description/config/motor_mapping.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)
        rospy.set_param("robot_model", robot_model)
        # if "DISPLAY" not in os.environ:
        #     Trajectory.RATE = 10000
        c = TrajectoryManagerRos()
        rospy.init_node("test")
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = trajectory_name
        msg.mirror = False
        joint_state = MagicMock()
        joint_state.position = [0.0] * 18
        rospy.wait_for_message = MagicMock(return_value=joint_state)
        c.command_callback(command=msg)
        c.run(real_time=real_time)

    @pytest.mark.parametrize("robot_model", ["bez1"])
    @pytest.mark.parametrize("trajectory_name", ["getupfront"])
    @pytest.mark.parametrize("real_time", [True])
    @unittest.skip("Not integrated in CI")
    def test_traj_ros(self, robot_model: str, trajectory_name: str, real_time: bool):
        self.run_real_trajectory(robot_model, trajectory_name, real_time)
