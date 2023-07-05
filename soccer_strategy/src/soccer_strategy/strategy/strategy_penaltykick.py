from soccer_msgs.msg import GameState
from soccer_strategy.strategy.strategy_freekick import StrategyFreekick
from soccer_strategy.team import Team


class StrategyPenaltykick(StrategyFreekick):
    """
    TODO Strategy used when a penalty kick is called
    """

    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().step_strategy(friendly_team, opponent_team, game_state)

        if friendly_team.observed_ball is None:
            return

        for robot in friendly_team.robots:
            pass
