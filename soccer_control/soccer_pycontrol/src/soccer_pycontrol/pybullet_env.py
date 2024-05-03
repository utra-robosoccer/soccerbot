import time

import numpy as np
import pybullet as pb
from soccer_pycontrol.joints import Joints
from soccer_pycontrol.links import Links
from soccer_pycontrol.pybullet_model import PybulletModel
from soccer_pycontrol.pybullet_world import PybulletWorld


class PybulletEnv:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(self, model: PybulletModel, world: PybulletWorld, real_time: bool = False, rate: int = 100):
        """
        Initialize the Navigator

        """
        self.rate = rate
        self.real_time = real_time

        self.world = world
        self.model = model

    def wait(self, steps) -> None:
        self.model.ready()
        self.interp_step()
        h = 0
        x, _, _ = self.model.ik.ik.x_sweep(h)
        for i in range(steps):
            self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            self.model.motor_control.set_motor()
            self.interp_step()

        for i in range(steps):
            self.step()

        x, _, _ = self.model.ik.ik.y_sweep(h)
        for i in range(steps):
            self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            self.model.motor_control.set_motor()
            self.interp_step()

        for i in range(steps):
            self.step()

        x, _, _ = self.model.ik.ik.z_sweep()
        for i in range(steps):
            self.model.motor_control.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = x[i][0:6]
            self.model.motor_control.set_motor()
            self.interp_step()

        for i in range(steps):
            self.step()

    def interp_step(self) -> None:
        # TODO this if for interpolation
        for _ in np.arange(0, 1.00, 0.040):
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()


if __name__ == "__main__":
    world = PybulletWorld(path="")
    model = PybulletModel(fixed_base=True)  # TODO dont know if i like this configuration
    p = PybulletEnv(model, world, real_time=True, rate=250)
    p.wait(50)
    p.world.close()
