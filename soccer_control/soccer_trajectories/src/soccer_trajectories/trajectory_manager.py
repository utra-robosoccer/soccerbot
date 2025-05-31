#!/usr/bin/env python3
import os
from abc import ABC, abstractmethod
from os.path import expanduser

from sensor_msgs.msg import JointState
from soccer_trajectories.trajectory import Trajectory


class TrajectoryManager(ABC):
    """
    Interfaces with trajectory and sends to pybullet
    """

    def __init__(self, robot_model: str, traj_name: str, mirror: bool = False):
        trajectory_path = expanduser("~") + f"/ros2_ws/src/soccerbot/soccer_control/soccer_trajectories/trajectories/{robot_model}/"

        self.trajectory_path = trajectory_path
        self.trajectory = Trajectory(trajectory_path + traj_name + ".csv", mirror=mirror)

    def process_trajectory(self, traj_name: str, mirror: bool):
        self.trajectory.trajectory_path = self.trajectory_path + traj_name + ".csv"
        self.trajectory.mirror = mirror
        self.trajectory.reset()

        self.trajectory.read_trajectory(self.read_joint_state())

    @abstractmethod
    def read_joint_state(self) -> JointState:
        pass

    @abstractmethod
    def send_trajectory(self, traj_name: str) -> None:
        pass

    @abstractmethod
    def send_joint_msg(self, timestamp: float) -> None:
        pass
