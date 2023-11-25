import math

import numpy as np
import py_trees.composites
import rospy
from Behaviours import *

from soccer_common.utils import wrapTo2Pi
from soccer_msgs.msg import GameState
from soccer_strategy.robot import Robot
from soccer_strategy.strategy.strategy import (
    Strategy,
    get_back_up,
    update_average_ball_position,
)
from soccer_strategy.strategy.utils import Utility
from soccer_strategy.team import Team


def create_tree():
    root = py_trees.composites.Selector("Dummber BT", memory=False)

    have_you_seen_the_ball = py_trees.composites.Selector("seen_ball", memory=False)
    have_you_seen_the_ball.add_children()

    # Base layer
    root.add_children(Check_Robot_Status_Non_Idle)  # Check if the robot is in any idle state
    root.add_children(Have_seen_ball)  # If we have seen ball
    root.add_children(Have_not_seen_ball)  # If we have not seen the ball

    return root


class StrategyDummer(Strategy):
    """
    A More basic strategy, we are not even thnking that much
    """

    def __init__(self):
        self.time_of_end_of_action = rospy.Time.now()
        self.update_frequency = 1
        super(StrategyDummer, self).__init__()

    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)

        this_robot = self.get_current_robot(friendly_team)
        tree = create_tree()

        tree.tick_once()
