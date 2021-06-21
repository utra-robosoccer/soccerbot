import enum

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
        READY = 1
        LOCALIZING = 2
        WALKING = 3
        KICKING = 4
        FALLEN_FRONT = 5
        FALLEN_BACK = 6
        PENALTY = 7
        OUT_OF_BOUNDS = 8
        TRAJECTORY_IN_PROGRESS = 9
        STOP = 10  #stop current action and getup
        TERMINATE = 11 #terminate all action

    def get_position(self):
        return self.position

    def __init__(self, team, role, status, position):
        self.team = team
        self.role = role
        self.status = status
        self.position = position
        self.goal_position = position

        self.speed = 0.20
        self.max_kick_speed = 2

    def set_navigation_position(self, position):
        self.goal_position = position

    def set_kick_velocity(self, kick_velocity):
        kick_angle_rand = np.random.normal(0, 0.2)
        kick_force_rand = np.random.normal(1, 0.5)

        rotation_rand = np.array([[np.cos(kick_angle_rand), -np.sin(kick_angle_rand)],
                                  [np.sin(kick_angle_rand), np.cos(kick_angle_rand)]])

        self.kick_velocity = kick_force_rand * rotation_rand @ kick_velocity

    def get_opponent_net_position(self):
        if self.team == Robot.Team.FRIENDLY:
            opponent_goal = np.array([0, 4.5])
        else:
            opponent_goal = np.array([0, -4.5])
        return opponent_goal


