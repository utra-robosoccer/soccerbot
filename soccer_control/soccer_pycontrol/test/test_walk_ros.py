import os
import time
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_ros.navigator_ros import NavigatorRos

from soccer_common import Transformation

os.environ["ROS_NAMESPACE"] = "/robot1"


class TestPybullet(unittest.TestCase):
    @unittest.skipIf("DISPLAY" not in os.environ, "only local")
    def test_walk_ros_local(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = NavigatorRos(bez)
        # walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        # walker.walk(d_x=0.04, t_goal=10)
        # target_goal = Transformation(position=[1, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
        target_goal = [0.04, 0, 0, 10, 500]
        walker.walk(target_goal)

        walker.wait(100)

    @unittest.skipIf("DISPLAY" not in os.environ, "only local")
    def test_walk_ros_webots(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )
        rospy.init_node("soccer_control")
        ns = "/robot1/"
        bez = BezROS(ns=ns)
        tmp = rospy.Publisher(ns + "reset_robot", PoseStamped)
        # tmp = rospy.Publisher(ns + "reset_robot", PoseStamped)
        walker = NavigatorRos(bez)
        walker.bez.motor_control.reset_target_angles()
        walker.bez.motor_control.set_motor()
        walker.wait(50)
        tmp.publish(PoseStamped())
        walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        start = time.time()
        # target_goal = Transformation(position=[1, 0, 0], euler=[0,0,0])
        # walker.walk(target_goal)
        walker.walk(bez.sensors.get_ball(), True)
        # target_goal = [0.0, 0, 0, 10, 500]
        # walker.walk(target_goal)
        print("cm/s: ", 100 / (time.time() - start))
        # walker.bez.sensors.get_ball(rospy.Time.now())
        walker.wait(100)
