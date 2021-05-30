#!/usr/bin/env python3
import rospy
from game_engine_ros import GameEngineRos

RUN_IN_ROS = False

if __name__ == '__main__':
    rospy.init_node("soccer_strategy")
    g = GameEngineRos()
    g.run()