from strategy.strategy import Strategy
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
import config
from strategy.interfaces.stop_resume_robots import StopResumeRobots

class ReadyStrategy(Strategy, StopResumeRobots):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.resume_all_robots(friendly_team)

        this_robot = self.get_current_robot(friendly_team)
        if this_robot.status == Robot.Status.LOCALIZING:
            # Continue to localize
            pass
        else:
            this_robot.set_navigation_position(config.FORMATIONS[this_robot.role])
