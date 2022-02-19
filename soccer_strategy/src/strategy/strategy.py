import math
import numpy as np

from robot import Robot
from robot_3d import Robot3D
from team import Team
from ball import Ball
from soccer_msgs.msg import GameState
import config

class Strategy():
    def __init__(self):
        pass

    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError


    def get_current_robot(self, friendly_team: Team):
        for robot in friendly_team.robots:
            if robot is Robot3D:
                return robot
        raise AssertionError

    def enemy_goal_position(self, game_state: GameState):
        return config.configure_position(config.ENEMY_GOAL_POSITION, game_state)


    def who_has_the_ball(self, robots, ball: Ball):
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

    def stop_all_robots(self, robots):
        for robot in robots:
            robot.stop_requested = True

    def resume_all_robots(self, robots):
        for robot in robots:
            robot.completed_trajectory_publisher.publish(True)
            if robot.stop_requested:
                robot.stop_requested = False
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY
