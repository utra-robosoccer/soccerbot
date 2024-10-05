import unittest

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.navigator import Navigator

from soccer_common import Transformation

REAL_TIME = True


class TestPlaco(unittest.TestCase):
    def tearDown(self):
        self.world.close()
        del self.bez
        del self.world

    def test_bez1(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        # walk.ready()
        # self.bez.motor_control.set_motor()
        # walk.wait(50)
        # target_goal = [0.08, 0, 0, 3, 500]
        target_goal = Transformation(position=[1, 1, 0], euler=[0, 0, 0])
        walk.walk(target_goal)
        # walk.wait(1000)

    # TODO fix
    def test_bez2(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="bez2", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        walk.walk(d_x=0.03, t_goal=10)

    def test_bez1_start_stop(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(100)
        walk.walk(d_x=0.03, t_goal=5)
        walk.wait(500)

    def test_bez1_replan(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez)
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
            walk.foot_step_planner.plan_steps(t)

            walk.bez.motor_control.configuration = walk.filter_joints()
            walk.stabilize_walk(pitch, roll)

            walk.bez.motor_control.set_motor()
            walk.world.step()

            t = walk.foot_step_planner.step(t)

    def test_bez1_ready(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        # TODO should bez be init in walk_engine
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        walk.ready()
        walk.world.step()
        walk.wait(100)
