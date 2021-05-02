#!/usr/bin/env python3
import os
import sys
from game_engine import GameEngine
from game_engine_ros import GameEngineRos
import rospy

RUN_IN_ROS=True

if __name__ == '__main__':
    if (len(sys.argv) > 2 and sys.argv[1] == '__name:=soccer_strategy') or RUN_IN_ROS:
        rospy.init_node("soccer_control")
        g = GameEngineRos()
        g.run()
    else:
        g = GameEngine()
        g.run()