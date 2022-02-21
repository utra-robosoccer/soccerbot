import abc
from robot_3d import Robot3D
from team import Team
from soccer_msgs.msg import GameState
import math

import numpy as np

import config
from ball import Ball
from robot import Robot
from team import Team

PLAYER_BALL_OFFSET = 0.1
NAVIGATION_BIAS = 1


class Strategy():

    @abc.abstractmethod
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError

    def get_current_robot(self, friendly_team: Team):
        for robot in friendly_team.robots:
            if robot is Robot3D:
                return robot
        raise AssertionError

    # Actions
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

    def kick(self, robot, ball):
        player_position = robot.position[0:2]
        ball_position = np.array(ball.position)
        player_angle = robot.position[2]
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

    def navigation_to_formation(self, team: Team, formation: str):
        for robot in team.robots:
            self.navigation_to_position(robot, config.FORMATIONS[formation][robot.role])

    def navigation_to_position(self, robot, position):
        robot.set_navigation_position(position)

    def navigate_to_position_with_offset(self, robot, ball):
        # generate destination pose
        ball_position = np.array(ball.position)
        goal_position = config.ENEMY_GOAL_POSITION[0:2]
        player_position = robot.position[0:2]

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

        self.navigation_to_position(robot, destination_position_biased)

    # Evaluations
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


    def can_kick_ball(self, robot: Robot, team: Team):
        player_position = robot.position[0:2]
        ball_position = np.array(team.average_ball_position.position)
        player_angle = robot.position[2]
        goal_position = np.array(config.ENEMY_GOAL_POSITION[0:2])
        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
                                     math.cos(player_angle - diff_angle))
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) < 0.15:
            return True
        return False

    def is_closest_to_ball(self, robot, friendly_team: Team):
        if friendly_team.average_ball_position != None:
            ball_position = np.array(friendly_team.average_ball_position.position)
            a = [np.linalg.norm(ball_position - robot.position[:2]) for robot in friendly_team.robots]
            closest = friendly_team.robots[np.argmin(a)]
            if robot == closest:
                return True
        return False


