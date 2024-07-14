import unittest

import numpy as np
import pybullet as pb
import pytest
from soccer_pycontrol.common.links import Links
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.walk_engine import WalkEngine

from soccer_common import Transformation


class TestPybullet(unittest.TestCase):

    def custom_walk(self, cameraTargetPosition: list, start_pose: Transformation, goal_pose: Transformation):
        world = PybulletWorld(camera_yaw=90, real_time=True, rate=100, cameraTargetPosition=cameraTargetPosition)
        bez = Bez(robot_model="bez1")
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
        bez = Bez(robot_model="bez1")
        tf = WalkEngine(world, bez)
        tf.wait(50)
        tf.ready()
        bez.motor_control.set_motor()
        world.wait_motor()
        tf.wait(50)
        tf.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        tf.walk()
        tf.wait(100)

    def test_walk_backward(self):
        self.custom_walk(
            cameraTargetPosition=[1, 0, 0.45], start_pose=Transformation([1, 0, 0], [0, 0, 0, 1]), goal_pose=Transformation([0.5, 0, 0], [0, 0, 0, 1])
        )

    def test_walk_2(self):
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=Transformation([-0.7384, -0.008, 0], [0, 0, 0, 1]),
            goal_pose=Transformation([0.0198, -0.0199, 0], [0, 0, 0, 1]),
        )

    def test_walk_3(self):
        # TODO currently wrong
        self.custom_walk(
            cameraTargetPosition=[-2, -1, 0.25],
            start_pose=Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139]),
            goal_pose=Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358]),
        )

    def test_walk_4(self):
        # TODO currently wrong
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=Transformation([0.3275415, 0.2841, 0.321], [0.04060593, 0.0120126, 0.86708929, -0.4963497]),
            goal_pose=Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953]),
        )

    def test_walk_5(self):
        # TODO currently wrong
        self.custom_walk(
            cameraTargetPosition=[0, 0, 0.25],
            start_pose=Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483]),
            goal_pose=Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176]),
        )

    def test_foot_step_planner_plane_dynamic(self):
        world = PybulletWorld(camera_yaw=90, real_time=True, rate=100)
        bez = Bez(robot_model="bez2")
        tf = WalkEngine(world, bez)
        tf.wait(50)
        tf.ready()
        tf.wait(50)
        for i in range(10):
            tf.set_goal(Transformation([0.2, 0, 0], [0, 0, 0, 1]))
            tf.walk()
        tf.wait(100)
        # TODO WIP need a way to figure out how to add more steps so it doesnt stop
