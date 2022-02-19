import config
import numpy as np
import math
from robot import Robot

class Kick():

    def kick(self, robot, ball):
        player_position = robot.get_position()[0:2]
        ball_position = np.array(ball.position)
        player_angle = robot.get_position()[2]
        goal_position = np.array(config.ENEMY_GOAL_POSITION["Positive"])
        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
                                     math.cos(player_angle - diff_angle))
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.18 and abs(
                nav_angle__diff) < 0.15 and robot.path.isFinished(robot.path_time):
            if nav_angle__diff > 0.03:
                # right foot
                robot.kick_with_right_foot = True
            else:
                robot.kick_with_right_foot = False

            delta = goal_position - ball_position
            unit = delta / np.linalg.norm(delta)

            robot.status = Robot.Status.KICKING
            robot.set_kick_velocity(unit * robot.max_kick_speed)


