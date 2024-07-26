import unittest

import numpy as np
import pybullet as pb
import pytest
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.old.links import Links
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine


class TestPybullet(unittest.TestCase):

    def test_imu(self):
        world = PybulletWorld(camera_yaw=45, real_time=True, rate=100)
        bez = Bez(robot_model="bez2")
        world.wait(100)
        for i in range(100):
            print(bez.sensors.get_imu().orientation_euler)
            world.step()
        # TODO add more

    def test_foot_sensor(self):
        world = PybulletWorld(camera_yaw=45, real_time=True, rate=100)
        bez = Bez(robot_model="bez2")
        world.wait(100)
        print(bez.sensors.get_foot_pressure_sensors(world.plane))
        # TODO add more and fuix the link location

    def test_bez_motor_range(self):
        world = PybulletWorld(path="", camera_yaw=90, real_time=True, rate=1000)
        bez = Bez(robot_model="bez2", fixed_base=True)
        world.wait(50)
        angles = np.linspace(-np.pi, np.pi)
        for i in range(bez.motor_control.numb_of_motors):

            for j in angles:
                x = [0.0] * bez.motor_control.numb_of_motors
                x[i] = j

                pb.setJointMotorControlArray(
                    bodyIndex=bez.model.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, bez.motor_control.numb_of_motors, 1)),
                    targetPositions=x,
                )
                world.wait_motor()

        world.wait(100)

    def test_bez_motor_range_single(self):
        world = PybulletWorld(path="", camera_yaw=90, real_time=True, rate=500)
        bez = Bez(robot_model="bez1", fixed_base=True)
        world.wait(50)
        angles = np.linspace(-np.pi, np.pi)

        for j in angles:
            x = [0.0] * bez.motor_control.numb_of_motors
            x[bez.data.motor_names.index("left_leg_motor_5")] = j
            x[bez.data.motor_names.index("right_leg_motor_5")] = j

            pb.setJointMotorControlArray(
                bodyIndex=bez.model.body,
                controlMode=pb.POSITION_CONTROL,
                jointIndices=list(range(0, bez.motor_control.numb_of_motors, 1)),
                targetPositions=x,
            )
            world.wait_motor()

        world.wait(100)

    def test_stand_plane(self):
        world = PybulletWorld(camera_yaw=0, real_time=True, rate=100)
        bez = Bez()
        tf = WalkEngine(world, bez)
        tf.wait(50)
        tf.bez.ready()
        tf.wait(50)
        while tf.t < 1000:
            [_, pitch, roll] = tf.bez.sensors.get_euler_angles()
            # if tf.fallen(pitch):
            #     pb.applyExternalForce(p.model.body, Links.TORSO, [0, 5, 0], [0, 0, 0], pb.LINK_FRAME)
            # p.model.set_pose()
            pb.applyExternalForce(tf.bez.model.body, Links.TORSO, [3, 0, 0], [0, 0, 0], pb.LINK_FRAME)
            tf.stabilize_stand(pitch, roll)
            tf.world.step()
            tf.t += +0.01
        tf.world.wait(100)


# @pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
# @pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_head_localizing_sweep(robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld(path="", camera_yaw=90, real_time=True, rate=1000, cameraTargetPosition=[0, 0, 0.25])
    bez = Bez(robot_model=robot_model, fixed_base=True)
    steps = 200
    thetas = bez.ik_actions.head_localizing_sweep()
    head_step = 0
    for i in range(steps):
        bez.motor_control.set_head_target_angles(thetas[i][:])
        bez.motor_control.set_motor()
        world.wait_motor()
        head_step += 1

    world.wait(steps)

    world.close()


@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_ready(robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """
    world = PybulletWorld(camera_yaw=45, real_time=True, rate=250)
    bez = Bez(robot_model=robot_model)

    world.wait(100)
    bez.ready()
    bez.motor_control.set_motor()
    world.wait_motor()
    world.wait(100)

    world.close()
    # TODO doesnt work with more then 1 robot model weird


@pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
@pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
@pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
def test_sweep(sweep_name: str, h: float, robot_model: str):
    """
    Case 1: Standard case
    :return: None
    """

    world = PybulletWorld(path="", camera_yaw=45, real_time=True, rate=1000, cameraTargetPosition=[0, 0, 0.25])
    bez = Bez(robot_model=robot_model, fixed_base=True)
    steps = 50
    x = np.zeros(steps)

    def sweep(target_pose: np.ndarray, step: int) -> None:
        for i in range(step):
            bez.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
            bez.motor_control.set_motor()
            world.wait_motor()

    if sweep_name == "x":
        x, _, _ = bez.ik_actions.x_sweep(h)
    elif sweep_name == "y":
        x, _, _ = bez.ik_actions.y_sweep(h)
    elif sweep_name == "z":
        x, _, _ = bez.ik_actions.z_sweep()

    sweep(x, steps)
    world.wait(steps)

    world.close()
