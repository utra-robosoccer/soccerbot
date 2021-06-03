#!/usr/bin/env python3
import sys
from game_engine import GameEngine

RUN_IN_ROS = False

if __name__ == '__main__':
    if (len(sys.argv) > 2 and sys.argv[1] == '__name:=soccer_strategy') and RUN_IN_ROS:
        import rospy
        from game_engine_ros import GameEngineRos
        rospy.init_node("soccer_strategy")
        g = GameEngineRos()
        g.run()
    else:
        DISPLAY_GAME = True
        NUM_GAMES = 1
        friendly_wins = 0
        opponent_wins = 0
        for i in range(NUM_GAMES):
            g = GameEngine(display=DISPLAY_GAME)
            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f'Friendly: {friendly_wins}, opponent: {opponent_wins}')
