import math
from typing import List, Optional

import numpy as np
import rospy
from soccer_object_detection.camera import Camera
from soccer_pycontrol import path

from soccer_common.transformation import Transformation
from soccer_common.utils import wrapToPi
from soccer_strategy.old.ball import Ball
from soccer_strategy.old.obstacle import Obstacle
from soccer_strategy.old.robot import Robot
from soccer_strategy.old.robot_controlled import RobotControlled


class RobotControlled2D(RobotControlled):
    class ObservationConstants:
        FOV = Camera.HORIZONTAL_FOV
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
        self.path = None
        self.trajectory_timeout = 0

    def __index__(self):
        return self.robot_id, self.team

    def __hash__(self):
        return hash((self.robot_id, self.team))

    def kick(self, kick_velocity):
        """
        Set's a kick velocity for the ball
        :param kick_velocity: Amount of strength for the kick
        :return:
        """
        self.status = Robot.Status.KICKING
        self.kick_velocity = kick_velocity

    def set_navigation_position(self, goal_position):
        super().set_navigation_position(goal_position)
        self.path = path.Path(Transformation(pos_theta=self.start_position), Transformation(pos_theta=self.goal_position))

    def get_opponent_net_position(self):
        if self.team == Robot.Team.FRIENDLY:
            opponent_goal = np.array([0, 4.5])
        else:
            opponent_goal = np.array([0, -4.5])
        return opponent_goal

    def observe_ball(self, ball: Optional[Ball]) -> bool:
        if ball is None or ball.position is None:
            return False
        theta = self.position[2]  # TODO change this to direction vector?
        ball_position = ball.position
        ball_diff = ball_position - self.position[0:2]
        angle = wrapToPi(theta - np.arctan2(ball_diff[1], ball_diff[0]))
        distance = np.linalg.norm(ball_diff)
        if abs(angle) < self.ObservationConstants.FOV / 2 and distance < self.ObservationConstants.VISION_RANGE:
            if self.observed_ball is None:
                self.observed_ball = Ball()
            self.observed_ball.position = ball_position
            self.observed_ball.last_observed_time_stamp = rospy.Time.now()
            self.robot_focused_on_ball_time = rospy.Time.now()

        # TODO can add noise here

    def observe_obstacles(self, robots: List[Robot]):
        self.observed_obstacles.clear()
        for robot in robots:
            if robot.robot_id != self.robot_id:
                theta = self.position[2]  # TODO change this to direction vector?
                arrow_len = 0.3
                arrow_end_x = math.cos(theta) * arrow_len
                arrow_end_y = math.sin(theta) * arrow_len
                robot_direction = np.array([arrow_end_x, arrow_end_y])
                obstacle_position = robot.position[0:2]
                obstacle_to_robot = np.array(obstacle_position) - np.array(self.position[0:2])
                angle = np.arccos(
                    np.dot(obstacle_to_robot[0:2], robot_direction) / (np.linalg.norm(obstacle_to_robot[0:2]) * np.linalg.norm(robot_direction))
                )
                distance = np.linalg.norm(obstacle_to_robot)
                if angle < self.ObservationConstants.FOV / 2 and distance < self.ObservationConstants.VISION_RANGE:
                    o = Obstacle()
                    o.position = robot.position[0:2]
                    o.team = robot.team
                    o.probability = (self.ObservationConstants.VISION_RANGE - distance) / self.ObservationConstants.VISION_RANGE
                    self.observed_obstacles.append(o)

    def get_robot_polygon(self):
        l = self.BODY_LENGTH
        w = self.BODY_WIDTH
        a = self.position[0:2]
        t = self.position[2]
        rotm = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
        p1 = a + rotm @ np.array([l, w])
        p2 = a + rotm @ np.array([l, -w])
        p3 = a + rotm @ np.array([-l, -w])
        p4 = a + rotm @ np.array([-l, w])

        return [p1, p2, p3, p4]
