#!/usr/bin/env python3

import psutil

# from soccer_pycontrol.model.model_ros.bez_ros import BezROS
# from soccer_pycontrol.walk_engine.walk_engine_ros.walk_engine_ros import WalkEngineROS

pid = psutil.Process()
try:
    pid.nice(-15)  # Will increase priority for a real robot run
except Exception as ex:
    pass

import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import numpy as np
import rospy
from soccer_pycontrol.navigator_ros import NavigatorRos

np.set_printoptions(precision=3)


if __name__ == "__main__":
    rospy.init_node("soccer_control")
    rospy.loginfo("Initializing Soccer Control")
    # bez = BezROS()
    # walker = WalkEngineROS(bez)
    walker = NavigatorRos()
    rospy.loginfo("Starting Control Loop")
    try:
        walker.run()
    except rospy.exceptions.ROSException as ex:
        exit(0)
