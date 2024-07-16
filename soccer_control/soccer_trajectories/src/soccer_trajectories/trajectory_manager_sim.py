#!/usr/bin/env python3

from sensor_msgs.msg import JointState
from soccer_trajectories.pybullet_setup import PybulletSetup
from soccer_trajectories.trajectory_manager import TrajectoryManager

from soccer_common import Transformation


class TrajectoryManagerSim(TrajectoryManager):
    """
    Interfaces with trajectory and sends to pybullet
    """

    def __init__(
        self, trajectory_path: str, pose: Transformation = Transformation(), robot_model: str = "bez1", real_time=False, mirror: bool = False, camera_yaw = 90
    ):
        super(TrajectoryManagerSim, self).__init__(trajectory_path, mirror)

        self.sim = PybulletSetup(robot_model=robot_model, real_time=real_time, pose=pose, camera_yaw=camera_yaw)

    def read_joint_state(self) -> JointState:
        return JointState(name=self.sim.motor_names, position=[0.0] * 20)

    def send_joint_msg(self, timestamp: float) -> None:
        states = [0.0] * 20
        joints, angles = self.trajectory.create_joint_states(timestamp)
        for i, joint in enumerate(joints):
            states[self.sim.motor_names.index(joint)] = angles[i]

        self.sim.motor_control(states)

    def send_trajectory(self, mirror: bool = False) -> None:
        self.process_trajectory(self.trajectory_path, mirror)

        t: float = 0
        while t <= self.trajectory.max_time:
            try:
                self.send_joint_msg(t)
            except Exception as ex:
                print(ex)
                exit(0)
            t += 1 / self.sim.rate

            print(t)

            self.sim.step()

        # self.sim.ramp.close()
        # self.trajectory.reset()
