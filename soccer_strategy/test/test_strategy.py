import os
import unittest
from os.path import expanduser
from random import uniform

import cv2
import numpy as np
import pybullet as pb
import pytest
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez, BezStatusEnum
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

from soccer_common import Transformation
from soccer_strategy.behavior.behavior_context import BehaviorContext
from soccer_strategy.behavior.state.get_up import GetUp
from soccer_strategy.behavior.state.walk import Walk
from soccer_strategy.behavior.head_state import FindBall
from soccer_strategy.behavior.state.balance import Balance

from soccer_msgs.msg import BoundingBox, BoundingBoxes


REAL_TIME = True
PLOT = True


class TestStrategy(unittest.TestCase):
    def tearDown(self):
        self.world.close()
        del self.bez
        del self.world

    def testwalktopose(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        nav = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        detect = None

        context = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True)

        target_goal = Transformation(position=[0.8, 0, 0], euler=[0, 0, 0])

        walk_state = Walk(target_goal)

        context.state = walk_state

        for i in range(1000):
            context.run_state_algorithim()  # calls walk_state.run_algorithim()
            self.world.step()

        self.assertTrue(True, "Completed the walk state without issues.")

    def testcontextswitch(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        nav = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        detect = None

        context = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True)

        target_goal = Transformation(position=[0.85, 0, 0], euler=[0, 0, 0])

        walk_state = Walk(target_goal)

        context.state = walk_state

        for i in range(8000):
            # can simulate behavior exec by checking is fallen on every iteration, testplaco auto
            context.run_state_algorithim()  # calls walk_state.run_algorithim()
            self.world.step()

        self.assertTrue(True, "Completed the walk state without issues.")

    def testforfall(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robozt_model="assembly", pose=Transformation())
        nav = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        detect = None

        context = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True)

        target_goal = Transformation(position=[0.85, 0, 0], euler=[0, 0, 0])

        walk_state = Walk(target_goal)

        context.state = walk_state

        for i in range(8000):
            # can simulate behavior exec by checking is fallen on every iteration, testplaco auto
            context.run_state_algorithim()  # calls walk_state.run_algorithim()
            y, p, r = self.bez.sensors.get_imu()

            if p > 1.25:
                context.transition_to(GetUp("getupfront"))
            elif p < -1.25:
                print("getupback: ")
                context.transition_to(GetUp("getupback"))
            elif r < -1.54 and -0.5 < p < -0.4:
                context.transition_to(GetUp("getupsideleft"))
            elif r > 1.54 and -0.5 < p < -0.4:
                context.transition_to(GetUp("getupsideright"))
            else:
                context.transition_to(Walk(target_goal))
            self.world.step()

        self.assertTrue(True, "Completed the walk state without issues.")

    #########################
    ##  HEAD STATE TESTS
    #########################

    def testfindball(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        nav = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        # detect = None

        context = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True)

        context.transition_to(FindBall())

        for i in range(8000):
            if i % 10 == 0:
                context.run_state_algorithim()
            self.world.step()

            # OBJECT DETECTION



    #########################
    ##  INTEGRATION TESTS
    #########################

    def testfindandchaseball(self):
        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        nav = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        detect = ObjectDetectionNode(model_path)

        target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])

        context_body = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True, head_state=False)
        context_head = BehaviorContext(self.world, self.bez, nav, tm, detect, sim=True, head_state=True)

        # Set up behaviors
        # context_body.transition_to(Balance())
        # context_body.body_state.context = context_body

        context_head.transition_to(FindBall())
        # context_head.head_state.context = context_head


        # Run both behaviors "in parallel"
        for i in range(8000): # order does not matter, if both called, body context has priority for some reason
            if i % 10 == 0:
                context_head.run_state_algorithim()
            context_body.run_state_algorithim()
            self.world.step()



        self.assertTrue(True, "Completed the walk state without issues.")