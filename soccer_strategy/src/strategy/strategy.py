import abc
try:
    from robot_controlled_3d import RobotControlled3D
except:
    print("not using robot controlled 3D")

from robot_controlled_2d import RobotControlled2D
from robot_controlled import RobotControlled

try:
    from soccer_msgs.msg import GameState
except:
    from soccer_msgs.fake_msg import GameState

from team import Team
from robot import Robot

def update_average_ball_position(update_next_strategy):
    def update_average_ball_position_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        friendly_team.update_average_ball_position()
        return update_next_strategy(self, friendly_team, opponent_team, game_state)
    return update_average_ball_position_strategy

def get_back_up(update_next_strategy):
    def get_back_up_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        current_robot = self.get_current_robot(friendly_team)
        if current_robot.status == Robot.Status.FALLEN_BACK:
            current_robot.run_fixed_trajectory("getupback")
            return
        elif current_robot.status == Robot.Status.FALLEN_FRONT:
            current_robot.run_fixed_trajectory("getupfront")
            return
        elif current_robot.status == Robot.Status.FALLEN_SIDE:
            current_robot.run_fixed_trajectory("getupside")
            return
        elif current_robot.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            return
        elif current_robot.status == Robot.Status.LOCALIZING:
            # Wait for localization status
            return
        return update_next_strategy(self, friendly_team, opponent_team, game_state)
    return get_back_up_strategy

class Strategy():
    def __init__(self):
        self.update_frequency = 1

    @abc.abstractmethod
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError

    def get_current_robot(self, friendly_team: Team) -> RobotControlled:
        for robot in friendly_team.robots:
            if type(robot) is RobotControlled2D:
                if robot.active:
                    return robot
            else:
                return robot

        raise AssertionError
