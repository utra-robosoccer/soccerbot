import numpy as np
from soccer_msgs.msg import GameState

import config as config
from strategy.strategy_freekick import StrategyFreekick
from team import Team


class StrategyPenaltykick(StrategyFreekick):
    # preparation if we are not the kicking team
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        if friendly_team.average_ball_position is None:
            return

        for robot in friendly_team.robots:
            pass