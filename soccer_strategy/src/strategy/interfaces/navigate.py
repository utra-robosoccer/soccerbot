import numpy as np
import config
import math

PLAYER_BALL_OFFSET = 0.1
NAVIGATION_BIAS = 1

class Navigate():

    def navigation_to_position(self, robot, position):
        robot.set_navigation_position(position)

    def navigate_to_position_with_offset(self, robot, ball):
        # generate destination pose
        ball_position = np.array(ball.position)
        goal_position = config.ENEMY_GOAL_POSITION
        player_position = robot.get_position()[0:2]

        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        destination_position = ball_position + diff_unit * PLAYER_BALL_OFFSET

        diff = destination_position - player_position
        # nav bias offset nav goal to be behind the ball
        destination_position_biased = player_position + diff * NAVIGATION_BIAS

        # nav goal behind the ball
        destination_position_biased = [destination_position_biased[0], destination_position_biased[1],
                                       diff_angle]

        self.navigation_to_position(position=destination_position_biased)

