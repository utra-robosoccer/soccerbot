import os
import unittest

import numpy as np
import pytest
from soccer_pycontrol.soccerbot.handle_urdf import HandleURDF
from soccer_pycontrol.soccerbot.pybullet_env import PybulletEnv
from soccer_pycontrol.soccerbot.pybullet_world import PybulletWorld

from soccer_common import Transformation


class TestPybullet(unittest.TestCase):
    def test_imu(self):
        world = PybulletWorld()
        model = HandleURDF()
        p = PybulletEnv(model, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_imu().orientation_euler)
        # TODO add more

    def test_foot_sensor(self):
        world = PybulletWorld()
        model = HandleURDF()
        p = PybulletEnv(model, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_foot_pressure_sensors(p.world.plane))
        # TODO add more


@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_ready(robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld()
    model = HandleURDF(robot_model=robot_model, pose=Transformation())
    p = PybulletEnv(model, world, real_time=True, rate=250)
    p.wait(100)
    p.motor_control.set_target_angles(p.ik_actions.ready())
    # TODO do i need this
    # self.motor_control.configuration_offset = [0] * len(Joints)

    p.wait_motor()
    p.wait(100)

    p.world.close()
    # TODO doesnt work with more then 1 robot model weird


@pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
@pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_sweep(sweep_name: str, h: float, robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld(path="")
    model = HandleURDF(fixed_base=True, robot_model=robot_model)
    p = PybulletEnv(model, world, real_time=True, rate=1000)

    steps = 50
    x = np.zeros(steps)

    def sweep(target_pose: np.ndarray, step: int) -> None:
        for i in range(step):
            p.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
            p.wait_motor()

    if sweep_name == "x":
        x, _, _ = p.ik_actions.x_sweep(h)
    elif sweep_name == "y":
        x, _, _ = p.ik_actions.y_sweep(h)
    elif sweep_name == "z":
        x, _, _ = p.ik_actions.z_sweep()

    sweep(x, steps)
    p.wait(steps)

    p.world.close()
