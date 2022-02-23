from ball import Ball
import enum
from robot import Robot

class FieldSide(enum.IntEnum):
    NORMAL = 0
    REVERSED = 1

class Team():

    def __init__(self, robots):

        self.robots = robots
        self.average_ball_position: Ball = None
        self.field_side = FieldSide.NORMAL
        self.is_first_half = False
        self.strategy = None
        self.formations = {
            "initial": {
                Robot.Role.GOALIE: [-1.5, 0, 0],
                Robot.Role.STRIKER: [-2, 0, 1.57],
                Robot.Role.RIGHT_WING: [-1, 2.5, 0],
                Robot.Role.LEFT_WING: [-1.5, 1.75, 0]
            },
            "attack": {
                Robot.Role.GOALIE: [4.5, 0, 0],
                Robot.Role.STRIKER: [-2, 0, 0],
                Robot.Role.RIGHT_WING: [-2, 2.5, 0],
                Robot.Role.LEFT_WING: [-2, -2.5, 0]
            },
            "defensive": {
                Robot.Role.GOALIE: [4.5, 0, 0],
                Robot.Role.STRIKER: [3.5, 0, 0],
                Robot.Role.RIGHT_WING: [3.5, 2, 0],
                Robot.Role.LEFT_WING: [3, -2, 0]
            },
            "midfield": {
                Robot.Role.GOALIE: [4.5, 0, 0],
                Robot.Role.STRIKER: [0, 0, 0],
                Robot.Role.RIGHT_WING: [0, 3, 0],
                Robot.Role.LEFT_WING: [0, -3, 0]
            },
            "penalty_give": {
                Robot.Role.GOALIE: [-4.5, 0, 0],
                Robot.Role.STRIKER: [3.5, 0, 0]
            },
            "penalty_take": {
                Robot.Role.GOALIE: [-4.5, 0, 0]
            }
        }
        self.enemy_goal_position = [4.8, 0]

    def flip_positions(self):
        self.enemy_goal_position[0] = -self.enemy_goal_position[0]
        for formation in self.formations:
            for role in self.formations[formation]:
                self.formations[formation][role][0] = -self.formations[formation][role][0]
                self.formations[formation][role][0] = -3.14

    def team_data_callback(self, data):
        robot = data[0]
        ball = data[1]
        if robot.robot_id in self.robots.keys():
            self.robots[robot.robot_id].robot_id = robot.robot_id
            self.robots[robot.robot_id].position = robot.position
            self.robots[robot.robot_id].covariance = None

            self.robots[robot.robot_id].observed_ball.position = ball.position

    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future
        ball_positions = []

        for robot in self.robots:
            # TODO
            pass

    def log(self):
        print("team_data:")
        pass