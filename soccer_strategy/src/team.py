from ball import Ball
import enum
import config

class FieldSide(enum.IntEnum):
    NORMAL = 0
    REVERSED = 1

class Team():

    def __init__(self, robots):

        self.robots = robots
        self.average_ball_position = Ball()
        self.field_side = FieldSide.NORMAL
        self.is_first_half = False
        self.formation = None
        self.strategy = None

        pass

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
        # TODO
