import os
import unittest

import rospy
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_ros.walk_engine_ros import WalkEngineROS

from soccer_common import Transformation

os.environ["ROS_NAMESPACE"] = "/robot1"


class TestPybullet(unittest.TestCase):

    def test_walk_ros_local(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = WalkEngineROS(bez)
        walker.wait(50)
        walker.ready()
        # bez.motor_control.set_motor()
        walker.wait(50)
        walker.set_goal(Transformation([1, 0, 0], [0, 0, 0, 1]), False)
        walker.run()
        print(walker.t)
        walker.wait(100)
