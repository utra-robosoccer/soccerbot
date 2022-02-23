from team import Team
from robot import Robot
from ball import Ball
import numpy as np
import config
import math

PLAYER_BALL_OFFSET = 0.1
NAVIGATION_BIAS = 1

class Actions:
    @staticmethod
    def stop_all_robots(robots: [Robot]):
        for robot in robots:
            robot.stop_requested = True

    @staticmethod
    def resume_all_robots(robots: [Robot]):
        for robot in robots:
            robot.completed_trajectory_publisher.publish(True)
            if robot.stop_requested:
                robot.stop_requested = False
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY

    @staticmethod
    def kick(robot: [Robot], ball: Ball):
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

    @staticmethod
    def navigation_to_formation(team: Team, formation: str):
        for robot in team.robots:
            Actions.navigation_to_position(robot, config.FORMATIONS[formation][robot.role])

    @staticmethod
    def navigation_to_position(robot, position):
        robot.set_navigation_position(position)

    @staticmethod
    def navigate_to_position_with_offset(robot, ball):
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

        Actions.navigation_to_position(robot, destination_position_biased)

