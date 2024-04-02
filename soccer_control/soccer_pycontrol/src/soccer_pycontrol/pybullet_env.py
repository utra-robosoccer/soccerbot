import time
from os.path import expanduser

import pybullet as pb
import pybullet_data
from soccer_pycontrol.pybullet_model import PybulletModel
from soccer_pycontrol.pybullet_world import PybulletWorld

from soccer_common import Transformation


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
        for i in range(steps):
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)
        pb.stepSimulation()


if __name__ == "__main__":
    world = PybulletWorld()
    model = PybulletModel(
        Transformation(position=(0, 0, 0.315)),
    )
    p = PybulletEnv(model, world, real_time=True)
    p.wait(1000)
    p.world.close()
