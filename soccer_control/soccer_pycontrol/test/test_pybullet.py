import unittest

import numpy as np
import pytest
from soccer_pycontrol.soccerbot2.kinematic_data import KinematicData
from soccer_pycontrol.soccerbot2.nav import Nav
from soccer_pycontrol.soccerbot2.pybullet.pybullet_env import PybulletEnv
from soccer_pycontrol.soccerbot2.pybullet.pybullet_world import PybulletWorld

from soccer_common import Transformation


class TestPybullet(unittest.TestCase):
    def test_imu(self):
        world = PybulletWorld()
        kinematic_data = KinematicData()
        p = PybulletEnv(kinematic_data, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_imu().orientation_euler)
        # TODO add more

    def test_foot_sensor(self):
        world = PybulletWorld()
        kinematic_data = KinematicData()
        p = PybulletEnv(kinematic_data, world, real_time=True, rate=100)
        p.wait(100)
        print(p.sensors.get_foot_pressure_sensors(p.world.plane))
        # TODO add more

    def test_foot_step_planner_fixed(self):
        world = PybulletWorld(path="")
        # TODO fix with torso height or start pose

        kinematic_data = KinematicData()
        p = PybulletEnv(kinematic_data, world, fixed_base=True, real_time=True, rate=100)
        tf = Nav(p)
        p.wait(50)
        tf.ready()
        p.wait(50)
        tf.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        tf.walk()
        p.wait(100)

    def test_foot_step_planner_plane(self):
        world = PybulletWorld()
        kinematic_data = KinematicData()
        p = PybulletEnv(kinematic_data, world, real_time=True, rate=100)
        tf = Nav(p)
        p.wait(50)
        tf.ready()
        p.wait(50)
        tf.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        tf.walk()
        p.wait(100)


@pytest.mark.parametrize("robot_model", ["bez1"])  # , "bez2"]) # TODO problem with bez2 urdf
def test_ready(robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld()
    kinematic_data = KinematicData(robot_model=robot_model)
    p = PybulletEnv(kinematic_data, world, real_time=True, rate=250)
    p.wait(100)
    p.motor_control.set_target_angles(p.ik_actions.ready())
    p.motor_control.set_motor()

    p.wait_motor()
    p.wait(100)

    p.world.close()
    # TODO doesnt work with more then 1 robot model weird


@pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
@pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
@pytest.mark.parametrize("robot_model", ["bez1"])  # , "bez2"])
def test_sweep(sweep_name: str, h: float, robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """

    world = PybulletWorld(path="", camera_yaw=45)
    kinematic_data = KinematicData(robot_model=robot_model)
    p = PybulletEnv(kinematic_data, world, fixed_base=True, real_time=True, rate=1000)

    steps = 50
    x = np.zeros(steps)

    def sweep(target_pose: np.ndarray, step: int) -> None:
        for i in range(step):
            p.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
            p.motor_control.set_motor()
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
