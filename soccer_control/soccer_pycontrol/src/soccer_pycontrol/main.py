#!/usr/bin/env python3

import psutil
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_ros.navigator_ros import NavigatorRos

pid = psutil.Process()
try:
    pid.nice(-15)  # Will increase priority for a real robot run
except Exception as ex:
    pass

import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import numpy as np
import rclpy

np.set_printoptions(precision=3)


if __name__ == "__main__":
    self.init_node("soccer_control")
    self.loginfo("Initializing Soccer Control")
    ns = "/robot1/"
    bez = BezROS(ns=ns)
    walker = NavigatorRos(bez)
    self.loginfo("Starting Control Loop")
    try:
        walker.walk(d_x=0.00, t_goal=10)
    except self.exceptions.ROSException as ex:
        exit(0)
