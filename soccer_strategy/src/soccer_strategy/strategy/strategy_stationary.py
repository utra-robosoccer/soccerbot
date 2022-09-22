from soccer_msgs.msg import GameState
from soccer_strategy.strategy.strategy import Strategy, get_back_up
from soccer_strategy.team import Team


class StrategyStationary(Strategy):
    """
    Stationary strategy, basically do nothing
    """

    @get_back_up
    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)

        pass
