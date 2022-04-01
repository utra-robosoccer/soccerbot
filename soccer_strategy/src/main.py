#!/usr/bin/env python3
import os
if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import rospy
from game_engine_3d import GameEngine3D

if __name__ == '__main__':
    rospy.init_node("soccer_strategy")
    g = GameEngine3D()
    try:
        g.run()
    except rospy.exceptions.ROSException as ex:
        print(ex)
        exit(0)
