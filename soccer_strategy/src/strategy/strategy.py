import abc
from robot_3d import Robot3D
from team import Team
from soccer_msgs.msg import GameState

class Strategy():

    @abc.abstractmethod
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError

    def get_current_robot(self, friendly_team: Team):
        for robot in friendly_team.robots:
            if robot is Robot3D:
                return robot
        raise AssertionError

