import unittest

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine


class TestPlaco(unittest.TestCase):
    def test_bez1(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEngine(world, bez)
        walk.ready()
        bez.motor_control.set_motor()
        walk.wait(50)
        walk.walk(d_x=0.04, t_goal=100)

    def test_bez2(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        bez = Bez(robot_model="bez2")
        walk = WalkEngine(world, bez)
        walk.walk(d_x=0.03, t_goal=10)

    def test_bez1_start_stop(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEngine(world, bez)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(100)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(500)

    def test_bez1_replan(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEngine(world, bez)
        walk.foot_step_planner.setup_walk(d_x=0.03)
        walk.pid.reset_imus()
        t = 0
        while t < 15:  # TODO needs end condition
            [_, pitch, roll] = walk.bez.sensors.get_imu()
            if t > 3:
                walk.foot_step_planner.configure_planner(d_y=0.03)
            if t > 6:
                walk.foot_step_planner.configure_planner(d_x=0.0, d_theta=0.4)
            if t > 9:
                walk.foot_step_planner.configure_planner(d_x=0.03, d_y=0.03)
            walk.foot_step_planner.walk_loop(t)

            walk.bez.motor_control.configuration = walk.filter_joints()
            walk.stabilize_walk(pitch, roll)

            walk.bez.motor_control.set_motor()
            walk.world.step()

            t = walk.foot_step_planner.step(t)

    def test_bez1_ready(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        # TODO should bez be init in walk_engine
        bez = Bez(robot_model="bez1")
        walk = WalkEngine(world, bez)
        walk.ready()
        walk.world.step()
        walk.wait(100)
