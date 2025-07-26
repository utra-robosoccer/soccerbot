#!/usr/bin/env python3
import os
from os.path import expanduser

import rclpy
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.navigator import Navigator

from soccer_common import Transformation
from soccer_strategy.behavior.behavior_context import BehaviorContext

REAL_TIME = True


class BehaviorExecutive:
    """
    This class is responsible for the main decision-making of the evtol and uses all systems to control the evtol.
    Integration and delegation for other modules. Code for any decision drone has to make.
    """

    def __init__(self):
        # Initialize node

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        self.nav = Navigator(self.world, self.bez, imu_feedback_enabled=False)

        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"
        self.detect = ObjectDetectionNode(model_path)  # TODO should have a switch

        self.behavior = BehaviorContext(world=self.world, bez=self.bez, nav=self.nav, detect=self.detect)

        self.nav.ready()
        self.nav.wait(100)

    # Main communication node for ground control
    def run(self):
        """
        Main loop

        :return: None
        """

        # Main loop to follow waypoints
        while not self.is_shutdown():
            # Behaviour Executive
            # TODO pass drone & path harder then previously thought might be possible but not worth time rigth now
            # self.behavior.run_state_algorithim()

            # AutoPilot
            # # TODO pass behavior
            # self._autopilot.check_autopilot()

            self.world.step()
