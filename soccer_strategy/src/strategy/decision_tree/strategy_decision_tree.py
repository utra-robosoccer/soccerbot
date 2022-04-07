import enum
import math

from robot import Robot
from strategy.strategy import Strategy
from team import Team

try:
    from soccer_msgs.msg import GameState
except:
    from fake_msg import GameState

import abc
from copy import deepcopy

import numpy as np


class TreeNode:
    def __init__(self, children):
        self.children = children

    def execute(self, robot, friendly_team, opponent_team, game_state):
        raise NotImplementedError("please implement")


class Decision(TreeNode):
    # returns true or false
    def execute(self, robot, friendly_team, opponent_team, game_state):
        raise NotImplementedError("please implement")


class Action(TreeNode):
    def __init__(self):
        super().__init__(None)
        return

    def execute(self, robot, friendly_team, opponent_team, game_state):
        raise NotImplementedError("please implement")


class GoToBall(Action):
    def execute(self, robot, friendly_team, opponent_team, game_state):
        # goal = [team_data.ball.position[0], team_data.ball.position[1], 3.14]
        # if not np.allclose(goal, robot.goal_position):
        #     robot.set_navigation_position(goal)
        robot.status = Robot.Status.WALKING
        goal_position = friendly_team.enemy_goal_position
        ball = friendly_team.average_ball_position
        ball_position = ball.position[0:2]
        player_position = robot.position[0:2]
        diff = ball_position - goal_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        distance_of_player_goal_to_ball = 0.1
        destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball

        navigation_bias = 1
        diff = destination_position - player_position
        # nav bias offset nav goal to be behind the ball
        destination_position_biased = player_position + diff * navigation_bias

        # nav goal behind the ball
        destination_position_biased = [destination_position_biased[0], destination_position_biased[1], diff_angle]
        robot.set_navigation_position(destination_position_biased)


class GoToFormationPosition(Action):
    def execute(self, robot, friendly_team, opponent_team, game_state):
        # TODO this fails when robot ID doesnt match, this is pre bad in general, should make a change to formation
        # player_position = robot.position
        # target_position = friendly_team.formation.closest_position(player_position).center
        target_position = friendly_team.formations["attack"][robot.role]
        robot.set_navigation_position(target_position)


class Shoot(Action):
    def execute(self, robot, friendly_team, opponent_team, game_state):
        goal_position = friendly_team.enemy_goal_position
        ball = friendly_team.average_ball_position
        delta = goal_position - ball.position[0:2]
        unit = delta / np.linalg.norm(delta)

        robot.status = Robot.Status.KICKING
        robot.set_kick_velocity(unit * robot.max_kick_speed)


class CanKick(Decision):
    def execute(self, robot, friendly_team, opponent_team, game_state):
        # #TODO should make navigation class to help with all this so the math doesn't need to be repeated everywhere
        # player_position = robot.position[0:2]
        # goal_position = friendly_team.enemy_goal_position
        # ball_position = friendly_team.average_ball_position.position[0:2]
        # player_angle = robot.position[2]
        # diff = ball_position - goal_position
        # diff_unit = diff / np.linalg.norm(diff)
        # diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])
        #
        # nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
        #                              math.cos(player_angle - diff_angle))
        # distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)
        #
        # if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) < 0.15:
        #     return True
        # return False
        return robot.can_kick(friendly_team.average_ball_position, friendly_team.enemy_goal_position)


class IsClosestToBall(Decision):
    def execute(self, robot, friendly_team, opponent_team, game_state):
        ball = friendly_team.average_ball_position
        if ball is not None:
            # TODO take into account rotation
            ball_position = ball.position[0:2]
            a = [np.linalg.norm(ball_position - robot.position[:2]) for robot in friendly_team.robots]
            closest = friendly_team.robots[np.argmin(a)]
            if robot.robot_id == closest.robot_id:
                return True
        return False


class DecisionTree:
    def __init__(self, root: TreeNode):
        self.root = root

    def execute(self, robot, friendly_team, opponent_team, game_state):
        curr = self.root
        while not isinstance(curr, Action):
            if curr.execute(robot, friendly_team, opponent_team, game_state):
                curr = curr.children[0]
            else:
                curr = curr.children[1]
        # action
        curr.execute(robot, friendly_team, opponent_team, game_state)


class FieldPosition:
    # can add other stuff like shape later
    def __init__(self, center):
        self.center = center


class FieldPositions:
    CENTER_STRIKER = FieldPosition(np.array([0, 0, np.pi]))
    GOALIE = FieldPosition(np.array([4.5, 0, np.pi]))
    RIGHT_WING = FieldPosition(np.array([-2, 3, np.pi]))
    LEFT_WING = FieldPosition(np.array([-2, -3, np.pi]))
    RIGHT_BACK = FieldPosition(np.array([3.5, 2, np.pi]))
    LEFT_BACK = FieldPosition(np.array([3.5, -2, np.pi]))
    CENTER_BACK = FieldPosition(np.array([3, 0, np.pi]))


# formations can be in a human readable data file?
class Formation:
    def __init__(self, positions):
        # array where 0 position is position for robot 0 (goalie) I guess?
        # TODO: change positions to dictionary with robot_id
        self.positions = positions

    def closest_position(self, target):
        a = [distance(position.center[:2], target[:2]) for position in self.positions]
        return self.positions[np.argmin(a)]


# should have a utils class for geometry calculations, geometry file/class?
def distance(o1, o2):
    return np.linalg.norm(o1 - o2)


class Formations:
    # don't go to role
    DEFENSIVE = Formation([FieldPositions.GOALIE, FieldPositions.LEFT_BACK, FieldPositions.RIGHT_BACK, FieldPositions.CENTER_BACK])
    ATTACKING = Formation([FieldPositions.GOALIE, FieldPositions.LEFT_WING, FieldPositions.CENTER_STRIKER, FieldPositions.RIGHT_WING])


from strategy.strategy import update_average_ball_position


class StrategyDecisionTree(Strategy):
    # can we just give pointers to friendly_teeam, opponent_team, and game_state here so we don't need to keep passing them?
    def __init__(self):
        super().__init__()
        self.current_formation = Formations.ATTACKING
        # have dictionary from robot role to decision tree
        self.decision_tree = DecisionTree(IsClosestToBall([CanKick([Shoot(), GoToBall()]), GoToFormationPosition()]))

    @update_average_ball_position
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        self.current_formation = self.decide_formation(friendly_team)
        friendly_team.formation = self.current_formation
        self.act_individual(friendly_team, opponent_team, game_state)

    # something so there can be different formation deciders like a super defensive biased one
    def decide_formation(self, friendly_team):
        return Formations.ATTACKING
        #
        # if self.friendly_team.average_ball_position.position is not None:  # ball in oppents side
        #     return Formations.ATTACKING
        # else:
        #     return Formations.DEFENSIVE

    def act_individual(self, friendly_team, opponent_team, game_state):
        robot = self.get_current_robot(friendly_team)
        self.decision_tree.execute(robot, friendly_team, opponent_team, game_state)
