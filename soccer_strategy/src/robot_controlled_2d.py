import math
import time

import numpy as np
from ball import Ball
from robot import Robot
from robot_controlled import RobotControlled


class RobotControlled2D(RobotControlled):
    class ObservationConstants:
        FOV = math.pi / 4
        VISION_RANGE = 3

    def __init__(
        self,
        robot_id=0,
        team=Robot.Team.UNKNOWN,
        role=Robot.Role.UNASSIGNED,
        status=Robot.Status.DISCONNECTED,
        position=None,
    ):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status, position=position)

        self.active = True  # Used only for switching which active robot for strategy stuff
        self.robot_id = robot_id
        self.robot_name = "robot %d" % robot_id
        self.path_time = 0

    def set_kick_velocity(self, kick_velocity):
        kick_angle_rand = np.random.normal(0, 0.2)
        kick_force_rand = max(np.random.normal(0.2, 0.3), 0)
        if kick_force_rand == 0:
            print("Kick Missed")

        rotation_rand = np.array([[np.cos(kick_angle_rand), -np.sin(kick_angle_rand)], [np.sin(kick_angle_rand), np.cos(kick_angle_rand)]])

        self.kick_velocity = kick_force_rand * rotation_rand @ kick_velocity

    def get_opponent_net_position(self):
        if self.team == Robot.Team.FRIENDLY:
            opponent_goal = np.array([0, 4.5])
        else:
            opponent_goal = np.array([0, -4.5])
        return opponent_goal

    def observe_ball(self, ball) -> bool:
        if ball is None or ball.position is None:
            return
        theta = self.position[2]  # TODO change this to direction vector?
        arrow_len = 0.3
        arrow_end_x = math.cos(theta) * arrow_len
        arrow_end_y = math.sin(theta) * arrow_len
        robot_direction = np.array([arrow_end_x, arrow_end_y])
        ball_position = ball.position
        ball_to_robot = ball_position - self.position[0:2]
        angle = np.arccos(np.dot(ball_to_robot[0:2], robot_direction) / (np.linalg.norm(ball_to_robot[0:2]) * np.linalg.norm(robot_direction)))
        distance = np.linalg.norm(ball_to_robot)
        if angle < self.ObservationConstants.FOV / 2 and distance < self.ObservationConstants.VISION_RANGE:
            self.observed_ball.position = ball_position
            self.navigation_goal_localized_time = time.time()

        # TODO can add noise here
