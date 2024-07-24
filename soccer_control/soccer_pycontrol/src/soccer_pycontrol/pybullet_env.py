import time
from os.path import expanduser

import numpy as np
import pybullet as pb
import pybullet_data
from matplotlib import pyplot as plt
from soccer_pycontrol.links import Links
from soccer_pycontrol.plot import BlitManager
from soccer_pycontrol.pybullet_model import PybulletModel
from soccer_pycontrol.pybullet_world import PybulletWorld
from soccer_pycontrol.soccerbot.inverse_kinematics import InverseKinematics

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

        # TODO cleanup
        # self.steps = []
        # self.imu_data = []
        #
        # x = np.linspace(0, 10, 100)
        # y = np.linspace(-1.57, 1.57, 100)
        # make a new figure
        # fig, self.ax = plt.subplots()
        # # add a line
        # (self.ln,) = self.ax.plot(x, y, animated=True)
        # # add a frame number
        # self.fr_number = self.ax.annotate(
        #     "0",
        #     (0, 1),
        #     xycoords="axes fraction",
        #     xytext=(10, -10),
        #     textcoords="offset points",
        #     ha="left",
        #     va="top",
        #     animated=True,
        # )
        # self.bm = BlitManager(fig.canvas, [self.ln, self.fr_number])
        # plt.show(block=False)
        # plt.pause(0.1)

    def wait(self, steps) -> None:
        # self.t_start = time.time()
        self.model.ready()
        for i in range(steps):

            print("Here: ", self.model.ik.get_link_transformation(Links.IMU, Links.RIGHT_LEG_6).position)
            print(pb.getLinkState(self.model.body, linkIndex=Links.RIGHT_LEG_6)[4])
            print(pb.getLinkState(self.model.body, linkIndex=Links.IMU)[4])
            self.step()

    def step(self) -> None:
        if self.real_time:
            time.sleep(1 / self.rate)

        # self.steps.append(len(self.steps) / self.rate)
        # self.imu_data.append(self.model.sensors.get_imu().orientation_euler[1])

        # TODO add to plot library
        # self.ln.set_data(self.steps, self.imu_data)
        # tx = "Mean Frame Rate:\n {fps:.3f}FPS".format(fps=((len(self.steps)) / (time.time() - self.t_start)))
        # self.fr_number.set_text(f"frame: {tx}")
        # self.bm.update()
        # self.ax.set_xlim(0, self.steps[-1], auto=True)
        # self.ax.set_ylim(min(self.imu_data), max(self.imu_data), auto=True)

        pb.stepSimulation()


if __name__ == "__main__":
    world = PybulletWorld(path="")
    model = PybulletModel(fixed_base=True)  # TODO dont know if i like this configuration
    p = PybulletEnv(model, world, real_time=True)
    p.wait(1000)
    p.world.close()
