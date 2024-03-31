import os
import unittest

from sensor_msgs.msg import JointState
from soccer_trajectories.trajectory import Trajectory
from soccer_trajectories.trajectory_manager_ros import TrajectoryManagerRos
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

from soccer_common import Transformation

os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

import pytest
import rospy

from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import FixedTrajectoryCommand


class TestTrajectory(unittest.TestCase):
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
        _, angles = traj.create_joint_states(traj.max_time)
        self.assertEqual(angles, [0.0, 0.0, 0.0, 0.0, 0.564, 0.564, -1.176, -1.176, 0.613, 0.613, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("trajectory_name", ["getupfront", "getupback", "rightkick"])
@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
@pytest.mark.parametrize("real_time", [False])
def test_trajectory_sim(trajectory_name: str, robot_model: str, real_time: bool):
    """
    Case 1: Standard case
    :return: None
    """
    if trajectory_name == "getupfront":
        pose = Transformation(position=[0, 0, 0.070], quaternion=[0.0, 0.707, 0.0, 0.707])
    elif trajectory_name == "getupback":
        pose = Transformation(position=[0, 0, 0.070], quaternion=[0.0, 0.707, 0.0, -0.707])
    else:
        pose = Transformation(position=[0, 0, 0.315], quaternion=[0.0, 0.0, 0.0, 1])

    print(os.path.join(os.path.dirname(__file__), "../trajectories/bez1_sim/" + trajectory_name + ".csv"))
    tm = TrajectoryManagerSim(
        os.path.join(os.path.dirname(__file__), "../trajectories/bez1_sim/" + trajectory_name + ".csv"), pose, robot_model, real_time
    )
    tm.send_trajectory()
    assert True
    # TODO add more testing from pybullet so like the height will reach a threshold and it doesnt fall over for
    #  some time also maybe split it per trajectory type


def run_real_trajectory(robot_model: str, trajectory_name: str, real_time: bool):
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

    c = TrajectoryManagerRos()
    rospy.init_node("test")
    msg = FixedTrajectoryCommand()
    msg.trajectory_name = trajectory_name
    msg.mirror = False

    joint_state = MagicMock()
    joint_state.position = [0.0] * 18
    rospy.wait_for_message = MagicMock(return_value=joint_state)

    c.command_callback(command=msg)
    c.send_trajectory(real_time=real_time)


@pytest.mark.parametrize("robot_model", ["bez1"])
@pytest.mark.parametrize("trajectory_name", ["getupfront"])
@pytest.mark.parametrize("real_time", [True])
@unittest.skip("Not integrated in CI")
def test_traj_ros(robot_model: str, trajectory_name: str, real_time: bool):
    run_real_trajectory(robot_model, trajectory_name, real_time)
