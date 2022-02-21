import numpy as np
from soccer_msgs.msg import GameState

import config as config
from strategy.freekick_strategy import FreekickStrategy
from team import Team


class PenaltykickStrategy(FreekickStrategy):
    # preparation if we are not the kicking team
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        if friendly_team.average_ball_position is None:
            return

        for robot in friendly_team.robots:
            pass