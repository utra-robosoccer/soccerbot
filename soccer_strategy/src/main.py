#!/usr/bin/env python3
import sys
import rospy
from game_engine import GameEngine

# useful sudo apt-get install -y python3-rospy

_IN_ROS = True

if _IN_ROS:
    from game_engine_competition import GameEngineCompetition

if __name__ == '__main__':
    if _IN_ROS:
        rospy.init_node("soccer_strategy")
        g = GameEngineCompetition()
        g.run()
    else:
        g = GameEngine()
        g.run_loop()


