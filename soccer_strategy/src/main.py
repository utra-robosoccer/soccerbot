#!/usr/bin/env python3
import sys
from game_engine import GameEngine
from game_engine_competition import GameEngineCompetition
import rospy
_IN_ROS = False

if __name__ == '__main__':
    if _IN_ROS:
        rospy.init_node("soccer_strategy")

        g = GameEngineCompetition()
        g.run()
    else:
        g = GameEngine()
        g.run()