import time

import numpy as np
import pybullet as pb
from soccer_pycontrol.soccerbot2.foot_step_planner import FootStepPlanner
from soccer_pycontrol.soccerbot2.ik.ik_actions import IKActions
from soccer_pycontrol.soccerbot2.kinematic_data import KinematicData
from soccer_pycontrol.soccerbot2.motor_control import MotorControl
from soccer_pycontrol.soccerbot2.pybullet.pybullet_world import PybulletWorld
from soccer_pycontrol.soccerbot2.pybullet_load_model import LoadModel
from soccer_pycontrol.soccerbot2.sensors import Sensors
from soccer_pycontrol.soccerbot2.walking_pid import WalkingPID

from soccer_common import Transformation


class PybulletEnv:
    """
    Sets up pybullet simulation for basic usage
    """

    # TODO update with the modified for pycontrol
    def __init__(
        self,
        kinematic_data: KinematicData,
        world: PybulletWorld,
        pose: Transformation = Transformation(),
        fixed_base: bool = False,
        real_time: bool = False,
        rate: int = 100,
    ):
        """
        Initialize the Navigator

        """

        self.rate = rate
        self.real_time = real_time

        self.world = world
        self.model = LoadModel(kinematic_data.urdf_model_path, kinematic_data.walking_torso_height, pose, fixed_base)

        self.motor_control = MotorControl(self.model.body)
        self.sensors = Sensors(self.model.body)

        self.ik_actions = IKActions(kinematic_data)

        self.step_planner = FootStepPlanner(self.model)
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
    pass
