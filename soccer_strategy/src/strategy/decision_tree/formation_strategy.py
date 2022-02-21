import enum

import config
from team import Team
from soccer_msgs.msg import GameState
from strategy.decision_tree_strategy import DecisionTreeStrategy, State, Action, Agent
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
class FormationState(State):
    def __init__(self, friendly_team=None, opponent_team=None, game_state=None):
        super(FormationState, self).__init__(friendly_team, opponent_team, game_state)


class FormationDecisionTreeStrategy(DecisionTreeStrategy):

    def PlayerStrategy(self, robot, team):
        isClosestToBall = lambda : self.is_closest_to_ball(robot, team)
        canKick = lambda : self.can_kick_ball(robot, team)

        if isClosestToBall():
            if canKick():
                return FormationAction.Action.KICK
            else:
                return FormationAction.Action.GO_TO_BALL
        else:
            return FormationAction.Action.GO_TO_FORMATION_POSITION


    def PredeterminedDecisionTree(self, state: State, currAgent: Agent, depth: int) -> (FormationAction, float):
        a = FormationAction()
        a.player_1_action = self.PlayerStrategy(state.friendly_team.robots[0], state.friendly_team)
        a.player_2_action = self.PlayerStrategy(state.friendly_team.robots[1], state.friendly_team)
        a.player_3_action = self.PlayerStrategy(state.friendly_team.robots[2], state.friendly_team)
        a.player_4_action = self.PlayerStrategy(state.friendly_team.robots[3], state.friendly_team)
        return a, 0

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
                self.navigation_to_position(robot, config.FORMATIONS[formation][robot.role])
            elif action == FormationAction.Action.KICK:
                self.kick(robot, state.friendly_team.average_ball_position)
            elif action == FormationAction.Action.GO_TO_BALL:
                self.navigate_to_position_with_offset(robot, state.friendly_team.average_ball_position)

        executeMove(state.friendly_team.robots[0], action.player_1_action)
        executeMove(state.friendly_team.robots[1], action.player_2_action)
        executeMove(state.friendly_team.robots[2], action.player_3_action)
        executeMove(state.friendly_team.robots[3], action.player_4_action)

    def getInitialState(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        return FormationState(friendly_team, opponent_team, game_state)

    DECISION_ALGORITHM = PredeterminedDecisionTree