from strategy.strategy import Strategy
from team import Team
from soccer_msgs.msg import GameState
from interfaces.actions import Actions

class StrategySet(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.stop_all_robots(friendly_team)
