import time
from typing import Union

import numpy as np
import pybullet as pb
from soccer_pycontrol.path.path_robot import PathRobot
from soccer_pycontrol.soccerbot2.foot_step_planner import FootStepPlanner
from soccer_pycontrol.soccerbot2.handle_urdf import HandleURDF
from soccer_pycontrol.soccerbot2.ik.ik_actions import IKActions
from soccer_pycontrol.soccerbot2.motor_control import MotorControl
from soccer_pycontrol.soccerbot2.pybullet.pybullet_world import PybulletWorld
from soccer_pycontrol.soccerbot2.sensors import Sensors
from soccer_pycontrol.soccerbot2.walking_pid import WalkingPID


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

        self.step_planner = FootStepPlanner(self.handle_urdf)
        self.pid = WalkingPID()

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

    p.world.close()
