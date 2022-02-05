from actions.action import Action
import numpy as np
import math
import config
from robot import Robot

class GoToBall(Action):
    def execute(self, robot, team_data):
        # goal = [team_data.ball.position[0], team_data.ball.position[1], 3.14]
        # if not np.allclose(goal, robot.goal_position):
        #     robot.set_navigation_position(goal)

        #generate destination pose
        robot.status = Robot.Status.WALKING
        ball_position = np.array(team_data.ball.position)
        #TODO enemy goal position should be stored in team data
        goal_position = np.array(config.ENEMY_GOAL_POSITION["Positive"])
        player_position = robot.get_position()[0:2]
        player_angle = robot.get_position()[2]

        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        distance_of_player_goal_to_ball = 0.1
        destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball

        navigation_bias = 1
        diff = destination_position - player_position
        # nav bias offset nav goal to be behind the ball
        destination_position_biased = player_position + diff * navigation_bias

        # nav goal behind the ball
        destination_position_biased = [destination_position_biased[0], destination_position_biased[1],
                                       diff_angle]

        if not np.allclose(destination_position_biased, robot.goal_position):
            robot.set_navigation_position(destination_position_biased)


