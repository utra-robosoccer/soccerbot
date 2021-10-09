import enum
from copy import deepcopy
import tf
from soccer_pycontrol import path
from soccer_pycontrol import transformation

import numpy as np


class Robot:
    class Team(enum.IntEnum):
        FRIENDLY = 1
        OPPONENT = 2

    class Role(enum.IntEnum):
        GOALIE = 1
        STRIKER = 2
        LEFT_MIDFIELD = 3
        RIGHT_MIDFIELD = 4

    class Status(enum.IntEnum):
        DISCONNECTED = 0
        READY = 1
        LOCALIZING = 2
        WALKING = 3
        KICKING = 4
        FALLEN_FRONT = 5
        FALLEN_BACK = 6
        FALLEN_SIDE = 7
        PENALTY = 8
        OUT_OF_BOUNDS = 9
        TRAJECTORY_IN_PROGRESS = 10
        STOPPED = 11  # Game controller

    def get_position(self):
        return self.position

    def __init__(self, robot_id, team, role, status, position):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.start_position = position
        self.goal_position = position
        self.path = None
        self.path_time = 0
        self.robot_id = robot_id

        self.speed = 0.20
        self.angular_speed = 0.3
        self.max_kick_speed = 2

    def set_navigation_position(self, position):
        self.status = Robot.Status.WALKING
        self.start_position = self.get_position()
        self.goal_position = position
        self.path = path.Path(
            self.position_to_transformation(self.start_position),
            self.position_to_transformation(self.goal_position)
        )
        self.path_time = 0

    def position_to_transformation(self, position):
        transfrom_position = (position[0], position[1], 0.)
        q = tf.transformations.quaternion_about_axis(position[2], (0, 0, 1))
        transform_quaternion = [q[0], q[1], q[2], q[3]]
        return transformation.Transformation(transfrom_position, transform_quaternion)

    def transformation_to_position(self, transform):
        transform_position = transform.get_position()
        transform_quaternion = transform.get_orientation()
        transfrom_angle = tf.transformations.euler_from_quaternion([
            transform_quaternion[0],
            transform_quaternion[1],
            transform_quaternion[2],
            transform_quaternion[3],
        ])

        return np.array([transform_position[0], transform_position[1], transfrom_angle[2]])

    def set_kick_velocity(self, kick_velocity):
        kick_angle_rand = np.random.normal(0, 0.2)
        kick_force_rand = 1#np.random.normal(1, 0.5)

        rotation_rand = np.array([[np.cos(kick_angle_rand), -np.sin(kick_angle_rand)],
                                  [np.sin(kick_angle_rand), np.cos(kick_angle_rand)]])

        self.kick_velocity = kick_force_rand * rotation_rand @ kick_velocity

    def get_opponent_net_position(self):
        if self.team == Robot.Team.FRIENDLY:
            opponent_goal = np.array([0, 4.5])
        else:
            opponent_goal = np.array([0, -4.5])
        return opponent_goal
