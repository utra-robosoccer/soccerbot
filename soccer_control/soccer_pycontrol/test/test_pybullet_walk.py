import unittest

import numpy as np
import pybullet as pb
import pytest
import scipy
from soccer_pycontrol.common.links import Links
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine

from soccer_common import Transformation


class TestPybullet(unittest.TestCase):

    def custom_walk(self, cameraTargetPosition: list, start_pose: Transformation, goal_pose: Transformation, robot_model: str = "bez2"):
        world = PybulletWorld(camera_yaw=90, real_time=True, rate=100, cameraTargetPosition=cameraTargetPosition)
        bez = Bez(robot_model=robot_model)
        tf = WalkEngine(world, bez)
        tf.bez.model.set_pose(start_pose)
        tf.wait(50)
        tf.ready()
        tf.wait(50)
        tf.set_goal(goal_pose)
        tf.walk()  # TODO add head rotation
        tf.wait(100)

    def test_foot_step_planner_fixed(self):
        world = PybulletWorld(path="", camera_yaw=90, real_time=True, rate=100, cameraTargetPosition=[0, 0, 0.25])
        bez = Bez(robot_model="bez1", fixed_base=True)
        tf = WalkEngine(world, bez)

        # TODO fix with torso height or start pose
        tf.wait(50)
        tf.ready()
        bez.motor_control.set_motor()
        world.wait_motor()
        tf.wait(50)
        tf.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        tf.walk()
        tf.wait(100)

    def test_foot_step_planner_plane(self):
        world = PybulletWorld(camera_yaw=90, real_time=True, rate=100, cameraTargetPosition=[0, 0, 0.45])
        bez = Bez(robot_model="bez2")
        tf = WalkEngine(world, bez)
        tf.wait(50)
        tf.ready()
        bez.motor_control.set_motor()
        world.wait_motor()
        tf.wait(50)
        tf.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        tf.walk()
        print(tf.t)
        tf.wait(100)

    def test_walk_2(self):
        start = Transformation([-0.7384, -0.008, 0], [0, 0, 0, 1])
        goal = Transformation([0.0198, -0.0199, 0], [0, 0, 0, 1])
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=start,
            goal_pose=goal,
        )

    def test_walk_3(self):
        start = Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139])
        goal = Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358])
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=start,
            goal_pose=goal,
        )

    def test_walk_4(self):
        # start = Transformation([0.3275415, 0.2841, 0.321], euler=(1.57,0.0, 0))
        # goal = Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953])
        start = Transformation([0.3275415, 0.2841, 0.321], [0.04060593, 0.0120126, 0.86708929, -0.4963497])
        goal = Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953])
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=start,
            goal_pose=goal,
        )

    def test_walk_5(self):
        start = Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483])
        goal = Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176])
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=start,
            goal_pose=goal,
        )

    def test_walk_side(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation([0, -0.5, 0], [0.00000, 0, 0, 1]),
        )

    def test_walk_backward(self):
        # TODO stabilize doesnt work
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation([-1, 0.3, 0], [0.00000, 0, 0, 1]),
        )

    def test_turn_in_place(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45], start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]), goal_pose=Transformation(euler=[np.pi, 0, 0])
        )

    # TODO fix later for new relative goal
    def test_small_movement_0(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation(position=[0.05, 0.05, 0], euler=[np.pi / 5, 0, 0]),
        )

    def test_small_movement_1(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation(position=[0.15, 0.05, 0], euler=[np.pi, 0, 0]),
        )

    def test_small_movement_2(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation(position=[-0.3, 0, 0], euler=[np.pi, 0, 0]),
        )

    def test_small_movement_3(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]),
            goal_pose=Transformation(position=[-0.2, -0.2, 0], euler=[-np.pi / 2, 0, 0]),
        )

    def test_small_movement_4(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0.2489, -0.163, 0.0], [0.0284, -0.003, 0.9939, 0.01986]),
            goal_pose=Transformation([0.0503, 0.06323, 0], [0, 0, 1, 0]),
        )

    def test_small_movement_5(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation(
                [0.3096807057334623, 0.09374110438873018, 0.0], [0.03189331238935847, -0.0065516868290173, 0.9990119776602083, 0.03024831426656374]
            ),
            goal_pose=Transformation([0.14076394628045208, -0.034574636811865296, 0], [0, 0, -0.9999956132297835, -0.002962013029887055]),
        )

    def test_do_nothing(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45], start_pose=Transformation([0, 0, 0], [0.00000, 0, 0, 1]), goal_pose=Transformation(euler=[0, 0, 0])
        )

    def test_walk_tiny_1(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0.0, 0, 0], [0, 0, 0, 1]),
            goal_pose=Transformation([0.01, 0, 0], [0, 0, 0, 1]),
        )

    def test_walk_tiny_2(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0.0, 0, 0], [0, 0, 0, 1]),
            goal_pose=Transformation([-0.01, 0, 0], [0, 0, 0, 1]),
        )

    def test_walk_tiny_3(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0.0, 0, 0], [0, 0, 0, 1]),
            goal_pose=Transformation([0.01, 0.01, 0], [0, 0, 0, 1]),
        )

    def test_walk_tiny_4(self):
        # TODO should be fine
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.45],
            start_pose=Transformation([0.0, 0, 0], [0, 0, 0, 1]),
            goal_pose=Transformation([0, 0, 0], euler=[0.1, 0, 0]),
        )

    def test_foot_step_planner_plane_dynamic(self):
        world = PybulletWorld(camera_yaw=90, real_time=True, rate=100)
        bez = Bez(robot_model="bez2")
        tf = WalkEngine(world, bez)
        tf.wait(50)
        tf.ready()
        tf.wait(50)
        for i in range(10):
            tf.set_goal(Transformation([0.5, 0, 0], [0, 0, 0, 1]), transform_global=False)
            tf.walk()
        tf.wait(100)
        # TODO WIP need a way to figure out how to add more steps so it doesnt stop
