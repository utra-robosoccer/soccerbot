from strategy.strategy import Strategy
from team import Team
from ball import Ball
from soccer_msgs.msg import GameState


class StrategyStationary(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        return