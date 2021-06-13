#!/usr/bin/env python3
import sys
from game_engine import GameEngine
from game_engine_competition import GameEngineCompetition
import rospy
_IN_ROS = True

if __name__ == '__main__':
    rospy.init_node("soccer_strategy")

    g = GameEngineCompetition()
    g.run()