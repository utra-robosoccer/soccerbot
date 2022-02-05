import numpy as np
import math
import config

PLAYER_BALL_OFFSET = 0.1
NAVIGATION_BIAS = 1

class Navigation:


    def navigate_to_position_with_offset(robot, team_data):
        # generate destination pose
        ball_position = np.array(team_data.ball.position)
        # TODO enemy goal position should be stored in team data
        goal_position = np.array(Navigation.get_goal_position(team_data))
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

        robot.set_navigation_position(destination_position_biased)


    def get_goal_position(team_data):
        goal_position = config.position_map_goal(
            config.ENEMY_GOAL_POSITION,
            team_data.field_side,
            team_data.is_first_half,
            1
        )

        if abs(team_data.ball.get_position()[1]) < 1.0:
            goal_position[1] = team_data.ball.get_position()[1]

        return goal_position
