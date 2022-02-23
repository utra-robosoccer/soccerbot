from ball import Ball
from robot import Robot
import math
import numpy as np
from team import Team
from robot import Robot


class Evaluations:
    # Evaluations
    @staticmethod
    def who_has_the_ball(robots: [Robot], ball: Ball) -> Robot:
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status != Robot.Status.READY:
                continue

            dist = np.linalg.norm(ball.position[0:2] - robot.position[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest

    @staticmethod
    def can_kick_ball(robot: Robot, team: Team) -> bool:
        player_position = robot.position[0:2]
        ball_position = np.array(team.average_ball_position.position)
        player_angle = robot.position[2]
        goal_position = np.array(team.enemy_goal_position)
        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
                                     math.cos(player_angle - diff_angle))
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) < 0.15:
            return True
        return False

    @staticmethod
    def is_closest_to_ball(robot: Robot, friendly_team: Team) -> bool:
        if friendly_team.average_ball_position != None:
            ball_position = np.array(friendly_team.average_ball_position.position)
            a = [np.linalg.norm(ball_position - robot.position[:2]) for robot in friendly_team.robots]
            closest = friendly_team.robots[np.argmin(a)]
            if robot == closest:
                return True
        return False
