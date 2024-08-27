import os
import unittest

import rospy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_ros.walk_engine_ros import WalkEngineRos

os.environ["ROS_NAMESPACE"] = "/robot1"


class TestPybullet(unittest.TestCase):

    def test_walk_ros_local(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = WalkEngineRos(bez)
        # walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        walker.walk(d_x=0.04, t_goal=10)
        walker.ready()
        # walker.walk(d_x=0.0, d_y=0.06, t_goal=5)
        # walker.walk(d_x=0.0, d_theta=0.3, t_goal=5)
        # walker.walk(d_x=-0.04, t_goal=5)
        # walker.walk(d_x=0.04, d_theta=0.2, t_goal=5)
        walker.wait(100)

    def test_walk_ros_webots(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )
        rospy.init_node("soccer_control")
        ns = "/robot1/"
        bez = BezROS(ns=ns)
        tmp = rospy.Publisher(ns + "reset_robot", PoseStamped)
        walker = WalkEngineRos(bez)
        walker.bez.motor_control.reset_target_angles()
        walker.bez.motor_control.set_motor()
        walker.wait(50)
        tmp.publish(PoseStamped())
        walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        walker.walk(d_x=0.05, t_goal=10)
        # walker.walk(d_x=0.0, d_y=0.06, t_goal=5)
        # walker.walk(d_x=0.0, d_theta=0.3, t_goal=5)
        # walker.walk(d_x=-0.04, t_goal=5)
        # walker.walk(d_x=0.04, d_theta=0.2, t_goal=5)
        walker.wait(100)
