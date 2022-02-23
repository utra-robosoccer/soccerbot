from strategy.strategy import Strategy
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
from strategy.interfaces.actions import Actions

class StrategyInitial(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.resume_all_robots(friendly_team.robots)

        current_robot = self.get_current_robot(friendly_team)
        current_robot.status = Robot.Status.LOCALIZING

        # TODO localize using goal posts
