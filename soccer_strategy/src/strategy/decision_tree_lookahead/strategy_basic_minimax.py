import enum

from team import Team
from soccer_msgs.msg import GameState
from strategy.decision_tree_lookahead.strategy_decision_tree_lookahead import (
    StrategyDecisionTreeLookhead,
    State,
    Action,
    Agent,
)
from strategy.interfaces.evaluations import Evaluations
from strategy.interfaces.actions import Actions
import numpy as np


class FormationAction(Action):
    class Action(enum.IntEnum):
        GO_TO_BALL = 0
        GO_TO_FORMATION_POSITION = 1
        KICK = 2
        STAND_STILL = 3

    def __init__(self):
        super().__init__()
        self.player_1_action = FormationAction.Action.STAND_STILL
        self.player_2_action = FormationAction.Action.STAND_STILL
        self.player_3_action = FormationAction.Action.STAND_STILL
        self.player_4_action = FormationAction.Action.STAND_STILL


# State
class FormationState(State, Evaluations):
    def __init__(self, friendly_team=None, opponent_team=None, game_state=None):
        super(FormationState, self).__init__(friendly_team, opponent_team, game_state)

    def getLegalActions(self, currAgent: Agent) -> []:
        if currAgent == Agent.FRIENDLY:
            team = self.friendly_team
        else:
            team = self.opponent_team

        def determinePlayerAction(robot):
            isClosestToBall = lambda: self.is_closest_to_ball(robot, self.friendly_team)
            canKick = lambda: self.can_kick_ball(robot, self.friendly_team)

            if isClosestToBall():
                if canKick():
                    return FormationAction.Action.KICK
                else:
                    return FormationAction.Action.GO_TO_BALL
            else:
                return FormationAction.Action.GO_TO_FORMATION_POSITION

        a = FormationAction()
        a.player_1_action = determinePlayerAction(team.robots[0])
        a.player_2_action = determinePlayerAction(team.robots[1])
        a.player_3_action = determinePlayerAction(team.robots[2])
        a.player_4_action = determinePlayerAction(team.robots[3])
        return [a]


class StrategyFormationDecisionTree(StrategyDecisionTreeLookhead):
    def executeBestMove(self, state: FormationState, action: FormationAction):
        # Decide formation
        formation = "defensive"
        ball_position = state.friendly_team.average_ball_position
        if ball_position is not None:
            if ball_position.position[0] < -1:  # ball in opponents side
                formation = "attacking"
            elif ball_position.position[0] < 1:
                formation = "midfield"
            else:
                formation = "defensive"
        else:
            formation = "defensive"

        def executeMove(robot, action: FormationAction.Action):
            if action == FormationAction.Action.STAND_STILL:
                pass
            elif action == FormationAction.Action.GO_TO_FORMATION_POSITION:
                Actions.navigation_to_position(robot, state.friendly_team.formations[formation][robot.role])
            elif action == FormationAction.Action.KICK:
                Actions.kick(robot, state.friendly_team.average_ball_position, state.friendly_team.enemy_goal_position)
            elif action == FormationAction.Action.GO_TO_BALL:
                Actions.navigate_to_position_with_offset(robot, state.friendly_team.average_ball_position, state.friendly_team.enemy_goal_position)

        executeMove(state.friendly_team.robots[0], action.player_1_action)
        executeMove(state.friendly_team.robots[1], action.player_2_action)
        executeMove(state.friendly_team.robots[2], action.player_3_action)
        executeMove(state.friendly_team.robots[3], action.player_4_action)

    def getInitialState(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        return FormationState(friendly_team, opponent_team, game_state)

    TREE_DEPTH = 1
