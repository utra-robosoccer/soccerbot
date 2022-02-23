from strategy.strategy import Strategy
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
from strategy.interfaces.actions import Actions

class StrategyReady(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        Actions.resume_all_robots(friendly_team)

        this_robot = self.get_current_robot(friendly_team)
        if this_robot.status == Robot.Status.LOCALIZING:
            # Continue to localize
            pass
        else:
            this_robot.set_navigation_position(friendly_team.formations[this_robot.role])
