#!/usr/bin/env python3
from abc import ABC, abstractmethod

from sensor_msgs.msg import JointState

from soccer_trajectories.trajectory import Trajectory


class TrajectoryManager(ABC):
    """
    Interfaces with trajectory and sends to pybullet
    """

    def __init__(self, trajectory_path: str, mirror: bool = False):
        self.trajectory_path = trajectory_path
        self.trajectory = Trajectory(trajectory_path, mirror=mirror)

    def process_trajectory(self, path: str, mirror: bool):
        self.trajectory.trajectory_path = path
        self.trajectory.mirror = mirror
        self.trajectory.reset()

        self.trajectory.read_trajectory(self.read_joint_state())

    @abstractmethod
    def read_joint_state(self) -> JointState:
        pass

    @abstractmethod
    def send_trajectory(self) -> None:
        pass

    @abstractmethod
    def send_joint_msg(self, timestamp: float) -> None:
        pass
