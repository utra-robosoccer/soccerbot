#!/usr/bin/env python3
import sys
import rospy
from game_engine import GameEngine
from game_engine_competition import GameEngineCompetition
# useful sudo apt-get install -y python3-rospy

_IN_ROS = True

if __name__ == '__main__':
    if _IN_ROS:
        rospy.init_node("soccer_strategy")
        g = GameEngineCompetition()
        g.run()
    else:
        g = GameEngine()
        g.run_loop()


