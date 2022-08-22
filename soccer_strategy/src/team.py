import enum
import os

import numpy as np
from ball import Ball
from robot import Robot


class FieldSide(enum.IntEnum):
    NORMAL = 0
    REVERSED = 1


# cartesian coordinates with field centered at (0,0), scoring in positive x direction
# http://humanoid.robocup.org/wp-content/uploads/RC-HL-2022-Rules-Changes-Marked-3.pdf
DEFAULT_FORMATIONS = {
    "ready": {
        Robot.Role.GOALIE: [-4, 0, 0],
        Robot.Role.STRIKER: [-0.5, 0, 0],
        Robot.Role.RIGHT_WING: [-1, 1, 0],
        Robot.Role.LEFT_WING: [-1, -1, 0],
    },
    "attack": {
        Robot.Role.GOALIE: [-4, 0, 0],
        Robot.Role.STRIKER: [2, 0, 0],
        Robot.Role.RIGHT_WING: [2, 2.5, 0],
        Robot.Role.LEFT_WING: [2, -2.5, 0],
    },
    "defensive": {
        Robot.Role.GOALIE: [-4, 0, 0],
        Robot.Role.STRIKER: [3.5, 0, 0],
        Robot.Role.RIGHT_WING: [3.5, 2, 0],
        Robot.Role.LEFT_WING: [3, -2, 0],
    },
    "midfield": {
        Robot.Role.GOALIE: [-4, 0, 0],
        Robot.Role.STRIKER: [0, 0, 0],
        Robot.Role.RIGHT_WING: [0, 3, 0],
        Robot.Role.LEFT_WING: [0, -3, 0],
    },
    "penalty_give": {Robot.Role.GOALIE: [-4, 0, 0], Robot.Role.STRIKER: [3.5, 0, 0]},
    "penalty_take": {Robot.Role.GOALIE: [-4, 0, 0]},
}


class Team:
    def __init__(self, robots):
        self.robots = robots
        self.average_ball_position: Ball = None
        self.field_side = FieldSide.NORMAL
        self.is_first_half = False
        self.strategy = None
        self.formation = None
        self.formations = DEFAULT_FORMATIONS
        self.enemy_goal_position = [4.5, 0]

    def flip_positions(self):
        self.enemy_goal_position[0] = -self.enemy_goal_position[0]
        for formation in self.formations:
            for role in self.formations[formation]:
                self.formations[formation][role][0] = -self.formations[formation][role][0]
                self.formations[formation][role][2] = -3.14

    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future, for now just use the current robot's position

        for robot in self.robots:
            if robot.robot_id == int(os.getenv("ROBOCUP_ROBOT_ID", 1)):
                self.average_ball_position = robot.observed_ball
                return True

        # Backup by getting any other robot's ball position
        for robot in self.robots:
            if robot.observed_ball is not None:
                self.average_ball_position = robot.observed_ball
                return True
        return False

    def log(self):
        np.set_printoptions(precision=3)
        for robot in self.robots:
            if robot.status != Robot.Status.DISCONNECTED:
                print(
                    "  Robot Id {}: Position: {}, Role: {}, Status: {}, Estimated Ball: {}".format(
                        robot.robot_id, robot.position, robot.role.name, robot.status.name, robot.observed_ball.position
                    )
                )
