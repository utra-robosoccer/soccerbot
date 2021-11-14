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


def unit_vec(v):
    return v / np.linalg.norm(v)


def distance_between(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)


def closest_point_on_line(pos, pt1, pt2):
    """
    Orthogonal projection of position pos onto the line from pt1 to pt2, as
    well as the distance to this point
    Source:
    - https://blender.stackexchange.com/questions/94464/finding-the-closest-point-on-a-line-defined-by-two-points
    """
    line_diff = pt2 - pt1
    # Compute distance
    proj = np.dot(pos, line_diff) / np.dot(line_diff, line_diff)
    dist = np.linalg.norm(line_diff * proj - pos)
    # Compute crit pos
    n = line_diff / np.linalg.norm(line_diff)
    ap = pos - pt1
    t = ap.dot(n)
    crit_pos = pt1 + t * n
    return crit_pos, dist


def tangent_lines_to_circle_through_pt(c, r, pt):
    """
    Given a circle centered at c with radius r, this function computes the
    points on the circle which lie on the tangent lines passing through the
    point pt
    First, we translate all points to the origin so that we only have to solve
    this one instance of the problem. The idea is to then write the equation of
    the circle, and substitute in y = k*x for some k. We then find the k-values
    that result in a repeated root for the quadratic circle equation. We
    substitute each of these k-values in to get the two different x coordinates
    then finally substitute the x,k pairs into y = k*x to get both points of
    interest.
    Returns left point, right point
    Source:
    - https://math.stackexchange.com/questions/543496/how-to-find-the-equation-of-a-line-tangent-to-a-circle-that-passes-through-a-g
    """
    x_c, y_c = c - pt  # Translate to origin and read out coordinates

    # Find values of k that result in a tangent line
    a_k = (r ** 2 - x_c ** 2)
    b_k = 2 * x_c * y_c
    c_k = (r ** 2 - y_c ** 2)
    disc = b_k ** 2 - 4 * a_k * c_k
    k1 = (-b_k - np.sqrt(disc)) / (2 * a_k)
    k2 = (-b_k + np.sqrt(disc)) / (2 * a_k)

    # Compute x values corresponding to each k
    a_x1 = 1 + k1 ** 2
    b_x1 = 2 * (x_c + k1 * y_c)
    a_x2 = 1 + k2 ** 2
    b_x2 = 2 * (x_c + k2 * y_c)
    c_x = x_c ** 2 + y_c ** 2 - r ** 2
    disc_x1 = b_x1 ** 2 - 4 * a_x1 * c_x
    disc_x2 = b_x2 ** 2 - 4 * a_x2 * c_x
    x1 = (-b_x1 - np.sqrt(max(disc_x1, 0))) / (2 * a_x1)
    x2 = (-b_x2 - np.sqrt(max(disc_x2, 0))) / (2 * a_x2)

    # Compute y values corresponding to each x value
    y1 = k1 * x1
    y2 = k2 * x2

    # Compute final points + translate back to original coordinates
    pt1 = np.array([x1, y1]) + pt
    pt2 = np.array([x2, y2]) + pt

    # Determine which point is "left" and which is "right". The "left" point
    # always has the greater slope
    if k1 > k2:
        ptl = pt1
        ptr = pt2
    else:
        ptl = pt2
        ptr = pt1
    return ptl, ptr


def func_att(alpha, x2, x1):
    return 0.5 * alpha * distance_between(x2, x1)


def grad_att(alpha, x2, x1):
    """Attractive gradient"""
    return alpha * (x2 - x1)


def func_rep(beta, r_rep, d_rep):
    val = 0
    if d_rep <= r_rep:
        t1 = (1 / d_rep) - (1 / r_rep)
        val = 0.5 * beta * t1 ** 2
    return val


def grad_rep(beta, r_rep, d_rep, x2, x1):
    """Repulsive gradient"""
    val = 0
    if d_rep <= r_rep:
        t1 = (1 / r_rep) - (1 / d_rep)
        t2 = (x2 - x1) / (2 * distance_between(x2, x1) ** 3)
        val = beta * t1 * t2
    return val


def distance_between(a, b):
    return np.linalg.norm(a - b)
