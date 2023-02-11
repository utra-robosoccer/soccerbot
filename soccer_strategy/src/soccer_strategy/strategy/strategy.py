import math

import numpy as np
import rospy

from soccer_common import Transformation
from soccer_msgs.msg import GameState
from soccer_strategy.ball import Ball
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled
from soccer_strategy.team import Team


def update_average_ball_position(update_next_strategy):
    """
    Decorator

    Calculates the average position of the ball according to the estimation from all the robots, at the moment it just
    takes the position of the controlling robot

    """

    def update_average_ball_position_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        friendly_team.update_average_ball_position()
        return update_next_strategy(self, friendly_team, opponent_team, game_state)

    return update_average_ball_position_strategy


def get_back_up(update_next_strategy):
    """
    Decorator that decorates a strategy that ensures that the robot gets back up when fallen
    """

    def get_back_up_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        current_robot = self.get_current_robot(friendly_team)
        if current_robot.status == Robot.Status.FALLEN_BACK:
            current_robot.run_fixed_trajectory("getupback")
            transform_current = Transformation(pos_theta=current_robot.position)
            transform_recovery = Transformation(position=[0, 0, 0])
            transform_new = transform_current @ transform_recovery
            current_robot.position = transform_new.pos_theta
            print(f"Updating Current Robot Position {transform_current.pos_theta} -> {transform_new.pos_theta}")
            return
        elif current_robot.status == Robot.Status.FALLEN_FRONT:
            current_robot.run_fixed_trajectory("getupfront")
            transform_current = Transformation(pos_theta=current_robot.position)
            transform_recovery = Transformation(position=[-0.5, 0, 0])
            transform_new = transform_current @ transform_recovery
            current_robot.position = transform_new.pos_theta
            print(f"Updating Current Robot Position {transform_current.pos_theta} -> {transform_new.pos_theta}")
            return
        elif current_robot.status == Robot.Status.FALLEN_SIDE:
            current_robot.run_fixed_trajectory("getupside")
            transform_current = Transformation(pos_theta=current_robot.position)
            transform_recovery = Transformation(position=[0, 0, 0])
            transform_new = transform_current @ transform_recovery
            current_robot.position = transform_new.pos_theta
            print(f"Updating Current Robot Position {transform_current.pos_theta} -> {transform_new.pos_theta}")
            return
        elif current_robot.status == Robot.Status.GETTING_BACK_UP:
            return
        elif current_robot.status == Robot.Status.LOCALIZING:
            # Wait for localization status
            return
        return update_next_strategy(self, friendly_team, opponent_team, game_state)

    return get_back_up_strategy


class Strategy:
    """
    The main strategy class
    """

    def __init__(self):
        self.update_frequency = 1  #: Indicates the frequency to run this strategy update
        self.iteration = 0  #: The number of times the strategy has been run
        self.complete = False  # Used to indicate that we can transition into a new strategy
        self.time_strategy_started = rospy.Time.now()

    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        """
        Runs a step in the strategy with the frequency update frequency

        :param friendly_team: The friendly Team
        :param opponent_team: The opponent Team
        :param game_state: The GameState from the game controller
        """
        self.iteration += 1

    def get_current_robot(self, friendly_team: Team) -> RobotControlled:
        """
        Returns the current robot from the friendly team

        :param friendly_team: The friendly team
        :return: The current robot
        """
        for robot in friendly_team.robots:
            if robot.__class__.__name__ == "RobotControlled3D":
                return robot
            if robot.__class__.__name__ == "RobotControlled2D" and robot.active == True:
                return robot

        raise AssertionError

    def who_has_the_ball(self, robots: [Robot], ball: Ball) -> Robot:
        """
        Gets the robot who has the ball, at the moment just uses the closest robot

        :param robots: List of robots to see which one has the ball
        :param ball: The ball object
        :return: The robot that is closest to the ball
        """
        closest_dist = math.inf
        current_closest = None
        for robot in robots:
            if robot.status not in [Robot.Status.READY, Robot.Status.WALKING, Robot.Status.KICKING, Robot.Status.GETTING_BACK_UP]:
                continue

            dist = np.linalg.norm(ball.position[0:2] - robot.position[0:2])
            if dist < closest_dist:
                closest_dist = dist
                current_closest = robot
        return current_closest
