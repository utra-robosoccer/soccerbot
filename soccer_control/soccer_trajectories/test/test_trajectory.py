import os
import unittest
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.node import Node

# import rclpy
from sensor_msgs.msg import JointState
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_trajectories.trajectory import Trajectory
from soccer_trajectories.trajectory_manager_ros import TrajectoryManagerRos
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

from soccer_common import Transformation

# from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import FixedTrajectoryCommand

os.environ["ROS_NAMESPACE"] = "/robot1"


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
            "right_hip_yaw",
            "left_hip_yaw",
            "right_hip_roll",
            "left_hip_roll",
            "right_hip_pitch",
            "left_hip_pitch",
            "right_knee",
            "left_knee",
            "right_ankle_pitch",
            "left_ankle_pitch",
            "right_ankle_roll",
            "left_ankle_roll",
            "right_shoulder_pitch",
            "left_shoulder_pitch",
            "right_shoulder_roll",
            "left_shoulder_roll",
            "head_yaw",
            "head_pitch",
        ]
        joint_state.position = [0.0] * 18

        traj.read_trajectory(joint_state)
        _, angles = traj.create_joint_states(traj.max_time)
        self.assertEqual(angles, [0.0, 0.0, 0.0, 0.0, 0.564, 0.564, -1.176, -1.176, 0.613, 0.613, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("trajectory_name", ["getupback"])  # getupside rightkick " getupback getupfront
@pytest.mark.parametrize("robot_model", ["assembly"])
@pytest.mark.parametrize("real_time", [True])
def test_trajectory_sim(trajectory_name: str, robot_model: str, real_time: bool):
    """
    Case 1: Standard case
    :return: None
    """
    # TODO update with pybullet updates
    camera = 0
    if "getupfront" in trajectory_name:
        pose = Transformation(position=[0, 0, 0.070], euler=[0, 1.57, 0])
        camera = 0
    elif "getupback" in trajectory_name:
        pose = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
        camera = 0
    elif "getupside" in trajectory_name:
        pose = Transformation(position=[0, 0, 0.13], euler=[0, 0, -1.57])
        camera = 0
    else:
        camera = 45
        pose = Transformation(position=[0, 0, 0.45], quaternion=[0.0, 0.0, 0.0, 1])
    print(os.path.join(os.path.dirname(__file__), "../trajectories/bez2_sim/" + trajectory_name + ".csv"))
    world = PybulletWorld(
        camera_yaw=camera,
        real_time=real_time,
        rate=200,
    )
    bez = Bez(robot_model=robot_model, pose=pose)

    tm = TrajectoryManagerSim(world, bez, "bez2_sim", "getupfront")
    tm.send_trajectory(trajectory_name)
    tm.world.wait(10000)
    tm.world.close()
    # TODO add more testing from pybullet so like the height will reach a threshold and it doesnt fall over for
    #  some time also maybe split it per trajectory type


def run_real_trajectory(robot_model: str, trajectory_name: str, real_time: bool):
    # TODO clean up
    # self.init_node("test")
    # os.system(
    #     "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy "
    #     "/robot1/soccer_pycontrol /robot1/soccer_trajectories'"
    # )

    rclpy.init()
    c = TrajectoryManagerRos(robot_model, trajectory_name)
    msg = FixedTrajectoryCommand()
    msg.trajectory_name = trajectory_name
    msg.mirror = False

    # TODO needs to be mocked, but not sure how
    # joint_state = MagicMock()
    # joint_state.position = [0.0] * 20
    # self.wait_for_message = MagicMock(return_value=joint_state)
    # Node.validate_topic_name= MagicMock(return_value=True)
    c.rate.sleep()
    c.command_callback(command=msg)
    c.send_trajectory(real_time=real_time)


# TODO clean up the pybullet interface so that there is a uniform place for the code


@pytest.mark.parametrize("robot_model", ["bez2"])
@pytest.mark.parametrize("trajectory_name", ["getupfront"])  # rightkick    rightkick_2  getupback_old
@pytest.mark.parametrize("real_time", [True])
@unittest.skipIf("DISPLAY" not in os.environ, "only local")
def test_traj_ros(robot_model: str, trajectory_name: str, real_time: bool):
    run_real_trajectory(robot_model, trajectory_name, real_time)
