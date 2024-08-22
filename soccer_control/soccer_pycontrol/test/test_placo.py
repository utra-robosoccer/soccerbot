import unittest

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.walk_engine_placo import WalkEnginePlaco


class TestPlaco(unittest.TestCase):

    def test_sigmaban(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=100,
        )
        bez = Bez(robot_model="sigmaban")
        walk = WalkEnginePlaco(world, bez)
        walk.walk(d_x=0.03, t_goal=10)

    def test_bez1(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=200,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEnginePlaco(world, bez)
        walk.ready()
        bez.motor_control.set_motor()
        walk.wait(50)
        walk.walk(d_x=0.04, t_goal=100)

    def test_bez2(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=100,
        )
        bez = Bez(robot_model="bez2")
        walk = WalkEnginePlaco(world, bez)
        walk.walk(d_x=0.03, t_goal=10)

    def test_bez1_start_stop(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=100,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEnginePlaco(world, bez)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(100)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(500)

    def test_bez1_replan(self):
        world = PybulletWorld(
            camera_yaw=90,
            real_time=True,
            rate=100,
        )
        bez = Bez(robot_model="bez1")
        walk = WalkEnginePlaco(world, bez)
        walk.walk_placo.setup_walk(d_x=0.03)
        walk.pid.reset_imus()
        t = 0
        while t < 15:  # TODO needs end condition
            [_, pitch, roll] = walk.bez.sensors.get_euler_angles()
            if t > 3:
                walk.walk_placo.configure_planner(d_y=0.03)
            if t > 6:
                walk.walk_placo.configure_planner(d_x=0.0, d_theta=0.4)
            if t > 9:
                walk.walk_placo.configure_planner(d_x=0.03, d_y=0.03)
            walk.walk_placo.walk_loop(t)

            joints = [0] * walk.bez.motor_control.numb_of_motors
            for joint in walk.bez.motor_control.motor_names:
                if joint in ["head_base_frame", "trunk_frame", "camera_frame", "left_foot_frame", "right_foot_frame", "/torso_imu"]:
                    continue
                joints[walk.bez.motor_control.motor_names.index(joint)] = walk.walk_placo.robot.get_joint(joint)

            walk.bez.motor_control.configuration = joints
            walk.stabilize_walk(pitch, roll)

            walk.bez.motor_control.set_motor()
            walk.world.step()

            t = walk.walk_placo.step(t)
