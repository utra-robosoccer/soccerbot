import math

import numpy as np
import rospy

from soccer_msgs.msg import GameState
from soccer_strategy.ball import Ball
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled
from soccer_strategy.team import Team


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


class Strategy:
    def __init__(self):
        self.update_frequency = 1
        self.iteration = 0
        self.complete = False  # Used to indicate that we can transition into a new strategy
        self.time_strategy_started = rospy.Time.now()

    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.iteration += 1

    def get_current_robot(self, friendly_team: Team) -> RobotControlled:
        for robot in friendly_team.robots:
            if robot.__class__.__name__ == "RobotControlled3D":
                return robot
            if robot.__class__.__name__ == "RobotControlled2D" and robot.active == True:
                return robot

        raise AssertionError

    def who_has_the_ball(self, robots: [Robot], ball: Ball) -> Robot:
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status not in [Robot.Status.READY, Robot.Status.WALKING, Robot.Status.KICKING]:
                continue

            dist = np.linalg.norm(ball.position[0:2] - robot.position[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest
