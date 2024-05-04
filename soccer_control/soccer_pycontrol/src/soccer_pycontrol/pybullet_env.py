import time

import numpy as np
import pybullet as pb
from soccer_pycontrol.motor_control import MotorControl
from soccer_pycontrol.pybullet_world import PybulletWorld
from soccer_pycontrol.sensors import Sensors
from soccer_pycontrol.soccerbot.handle_urdf import HandleURDF
from soccer_pycontrol.soccerbot.ik_actions import IKActions


class PybulletEnv:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, model: HandleURDF, world: PybulletWorld, real_time: bool = False, rate: int = 100):
        """
        Initialize the Navigator

        """
        self.rate = rate
        self.real_time = real_time

        self.world = world
        self.handle_urdf = model
        self.body = self.handle_urdf.body

        self.motor_control = MotorControl(self.body)
        self.sensors = Sensors(self.body)

        self.ik_actions = IKActions(self.handle_urdf.ik_data)

    def action(self, steps: int) -> None:
        self.motor_control.set_target_angles(self.ik_actions.ready())
        # TODO do i need this
        # self.motor_control.configuration_offset = [0] * len(Joints)

        self.wait_motor()
        self.wait(steps)
        h = 0

        x, _, _ = self.ik_actions.x_sweep(h)
        self.sweep(x, steps)
        self.wait(steps)

        x, _, _ = self.ik_actions.y_sweep(h)
        self.sweep(x, steps)
        self.wait(steps)

        x, _, _ = self.ik_actions.z_sweep()
        self.sweep(x, steps)
        self.wait(steps)

    def sweep(self, target_pose: np.ndarray, steps: int) -> None:
        for i in range(steps):
            self.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
            self.wait_motor()

    def wait(self, steps: int) -> None:
        for i in range(steps):
            self.step()

    def wait_motor(self) -> None:
        # TODO this if for interpolation
        for _ in np.arange(0, 1.00, 0.040):
            self.step()

    # TODO maybe put into seperate part
    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()


if __name__ == "__main__":
    world = PybulletWorld(path="")
    model = HandleURDF(fixed_base=True)
    p = PybulletEnv(model, world, real_time=True, rate=250)
    p.action(50)

    p.world.close()
