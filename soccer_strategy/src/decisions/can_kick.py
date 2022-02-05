import config
from decisions.decision import Decision
import math
import numpy as np

class CanKick(Decision):
    def execute(self, robot, team_data):
        #TODO should make navigation class to help with all this so the math doesn't need to be repeated everywhere
        player_position = robot.get_position()[0:2]
        ball_position = np.array(team_data.ball.position)
        player_angle = robot.get_position()[2]
        goal_position = np.array(config.ENEMY_GOAL_POSITION["Positive"])
        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
                                     math.cos(player_angle - diff_angle))
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) < 0.15:
            return True
        return False


