#!/usr/bin/env python3
import os
from typing import Optional

from sensor_msgs.msg import JointState

from soccer_trajectories.pybullet_setup import PybulletSetup
from soccer_trajectories.trajectory import Trajectory


class TrajectoryManager:
    def __init__(self, trajectory_path: str, robot_model: str, mirror=False):
        self.sim = PybulletSetup(robot_model)

        self.trajectory_path = trajectory_path

        self.trajectory = Trajectory(self.trajectory_path, mirror)
        joint_state = JointState(name=self.sim.motor_names, position=[0.0] * 18)

        self.trajectory.read_trajectory(joint_state)

    def run(self, real_time=True):
        t = 0
        while t <= self.trajectory.max_time + 0.01 and not self.trajectory.terminate:
            try:
                states = self.trajectory.create_pybullet_states(t, self.sim.motor_names)

                self.sim.motor_control(states)
            except Exception as ex:
                print(ex)
                exit(0)
            t += 0.01

            self.sim.step()


if __name__ == "__main__":
    tm = TrajectoryManager(os.path.join(os.path.dirname(__file__), "../../trajectories/bez1_sim/getupfront.csv"),
                           "bez1")
    tm.run()
