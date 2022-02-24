
from ball import Ball
import math
import numpy as np
from robot import Robot
from robot_controlled import RobotControlled


class RobotControlled2D(RobotControlled):

    class ObservationConstants:
        FOV = math.pi/4
        VISION_RANGE = 3

    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED, position=None):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status, position=position)

        self.robot_id = robot_id
        self.observed_ball = Ball(None)
        self.speed = 0.20
        self.angular_speed = 0.3
        self.max_kick_speed = 2
        self.robot_name = 'robot %d' % robot_id

    def set_kick_velocity(self, kick_velocity):
        kick_angle_rand = np.random.normal(0, 0.2)
        kick_force_rand = np.random.normal(1, 0.4)
        miss_ball = np.random.uniform(0, 1)
        if miss_ball < 0.2:
            kick_force_rand = 0

        rotation_rand = np.array([[np.cos(kick_angle_rand), -np.sin(kick_angle_rand)],
                                  [np.sin(kick_angle_rand), np.cos(kick_angle_rand)]])

        self.kick_velocity = kick_force_rand * rotation_rand @ kick_velocity


    def get_opponent_net_position(self):
        if self.team == Robot.Team.FRIENDLY:
            opponent_goal = np.array([0, 4.5])
        else:
            opponent_goal = np.array([0, -4.5])
        return opponent_goal

    def observe_ball(self, ball):
        if ball is None or ball.position is None:
            return
        theta = self.position[2] #TODO change this to direction vector?
        arrow_len = 0.3
        arrow_end_x = math.cos(theta) * arrow_len
        arrow_end_y = math.sin(theta) * arrow_len
        robot_direction = np.array([arrow_end_x, arrow_end_y])
        ball_position = ball.position
        ball_to_robot = ball_position - self.position[0:2]
        angle = np.arccos(np.dot(ball_to_robot[0:2], robot_direction)/(np.linalg.norm(ball_to_robot[0:2])*np.linalg.norm(robot_direction)))
        distance = np.linalg.norm(ball_to_robot)
        if angle < self.ObservationConstants.FOV/2 and distance < self.ObservationConstants.VISION_RANGE:
            self.observed_ball.position = ball_position
        #TODO can add noise here

    def can_kick(self, ball):
        if ball is None or ball.position is None:
            return False
        theta = self.position[2]
        robot_direction = np.array([math.cos(theta), math.sin(theta)])
        robot_direction = robot_direction/np.linalg.norm(robot_direction)
        ball_position = ball.position
        ball_to_robot = (ball_position - self.position[0:2])
        angle = np.arccos(np.dot(ball_to_robot, robot_direction)/(np.linalg.norm(ball_to_robot)*np.linalg.norm(robot_direction)))
        distance = np.linalg.norm(ball_to_robot)
        if angle < 0.1 and distance < 0.2: #TODO change constants to depend on something
            return True



