from soccer_msgs.msg import GameState
from soccer_strategy.strategy.strategy import Strategy, get_back_up
from soccer_strategy.team import Team


class StrategySet(Strategy):
    """
    Strategy used when set is called, do nothing and wait for the human to place the ball
    """

    def __init__(self):
        super().__init__()
        self.update_frequency = 5

    @get_back_up
    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)
