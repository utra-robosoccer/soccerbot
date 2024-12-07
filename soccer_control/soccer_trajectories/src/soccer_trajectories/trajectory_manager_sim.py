#!/usr/bin/env python3
import time

from sensor_msgs.msg import JointState
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_trajectories.trajectory_manager import TrajectoryManager

from soccer_common import Transformation


class TrajectoryManagerSim(TrajectoryManager):
    """
    Interfaces with trajectory and sends to pybullet
    """

    def __init__(
        self,
        trajectory_path: str,
        pose: Transformation = Transformation(),
        robot_model: str = "bez1",
        real_time=False,
        mirror: bool = False,
        camera_yaw=90,
    ):
        super(TrajectoryManagerSim, self).__init__(trajectory_path, mirror)
        self.world = PybulletWorld(
            camera_yaw=camera_yaw,
            real_time=real_time,
            rate=75,
        )
        self.bez = Bez(robot_model=robot_model, pose=pose)

    def read_joint_state(self) -> JointState:
        return JointState(name=list(self.bez.motor_control.motor_names.keys()), position=[0.0] * self.bez.motor_control.numb_of_motors)

    def send_joint_msg(self, timestamp: float) -> None:
        joints, angles = self.trajectory.create_joint_states(timestamp)
        for i, joint in enumerate(joints):
            self.bez.motor_control.configuration[joint] = angles[i]

        self.bez.motor_control.set_motor()

    def send_trajectory(self, mirror: bool = False) -> None:
        self.process_trajectory(self.trajectory_path, mirror)

        t: float = 0
        while t <= self.trajectory.max_time:
            try:
                self.send_joint_msg(t)
            except Exception as ex:
                print(ex)
                exit(0)
            t += 1 / self.world.rate

            print("time", t)

            self.world.step()
            # time.sleep(0.01)
        # self.sim.ramp.close()
        # self.trajectory.reset()
