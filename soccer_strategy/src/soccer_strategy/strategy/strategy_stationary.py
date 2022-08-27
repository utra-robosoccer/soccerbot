from soccer_msgs.msg import GameState
from soccer_strategy.ball import Ball
from soccer_strategy.strategy.strategy import Strategy, get_back_up
from soccer_strategy.team import Team


class StrategyStationary(Strategy):
    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)

        pass
