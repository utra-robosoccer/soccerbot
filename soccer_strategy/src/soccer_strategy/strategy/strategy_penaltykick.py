from soccer_msgs.msg import GameState
from soccer_strategy.strategy.strategy_freekick import StrategyFreekick
from soccer_strategy.team import Team


class StrategyPenaltykick(StrategyFreekick):
    # preparation if we are not the kicking team
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)

        if friendly_team.average_ball_position is None:
            return

        for robot in friendly_team.robots:
            pass