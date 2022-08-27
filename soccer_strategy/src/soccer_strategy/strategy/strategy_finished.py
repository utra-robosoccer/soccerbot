from soccer_msgs.msg import GameState
from soccer_strategy.strategy.interfaces.actions import Actions
from soccer_strategy.strategy.strategy import Strategy
from soccer_strategy.team import Team


class StrategyFinished(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)
        Actions.stop_all_robots(friendly_team)
