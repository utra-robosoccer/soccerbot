import numpy as np
from shapely.geometry import LineString, Point, Polygon
from robot import Robot


class GameProperties():
    def __init__(self, team_color, is_first_half, secondary_state, opponent_team=False):
        self.team_color = team_color
        self.is_first_half = is_first_half
        self.secondary_state = secondary_state
        self.opponent_team = opponent_team


class Field:
    X_COORD = 0
    Y_COORD = 1
    MIDFIELD = np.array([0, 0])
    # x axis
    X_LENGTH = 9
    X_NEGATIVE_BOUND = -4.5
    X_POSITIVE_BOUND = 4.5

    # y axis
    Y_LENGTH = 6
    Y_NEGATIVE_BOUND = -3
    Y_POSITIVE_BOUND = 3

    GOAL_WIDTH = 2.6
    GOAL_AREA_Y = 3
    GOAL_AREA_X = 1
    PENALTY_AREA_LENGTH = 2
    NET = {}
    NET_AREA = {}
    NET_LINE = {}

    @staticmethod
    def init():
        hw = Field.GOAL_WIDTH / 2
        # Friendly net
        x_goal_line = Field.X_POSITIVE_BOUND
        x_goal_area_line = Field.X_POSITIVE_BOUND - Field.GOAL_AREA_X
        goal_coords = [(x_goal_line, -hw), (x_goal_line, hw), (x_goal_area_line, -hw), (x_goal_area_line, hw)]
        Field.NET[Robot.Team.FRIENDLY] = Point(Field.X_POSITIVE_BOUND, 0)
        Field.NET_AREA[Robot.Team.FRIENDLY] = Polygon(goal_coords)
        Field.NET_LINE[Robot.Team.FRIENDLY] = LineString([(x_goal_line, -hw), (x_goal_line, hw)])

        # Opponent net
        x_goal_line = Field.X_NEGATIVE_BOUND
        x_goal_area_line = Field.X_NEGATIVE_BOUND + Field.GOAL_AREA_X
        goal_coords = [(x_goal_line, -hw), (x_goal_line, hw), (x_goal_area_line, -hw), (x_goal_area_line, hw)]
        Field.NET[Robot.Team.OPPONENT] = Point(Field.X_NEGATIVE_BOUND, 0)
        Field.NET_AREA[Robot.Team.OPPONENT] = Polygon(goal_coords)
        Field.NET_LINE[Robot.Team.OPPONENT] = LineString([(x_goal_line, -hw), (x_goal_line, hw)])
