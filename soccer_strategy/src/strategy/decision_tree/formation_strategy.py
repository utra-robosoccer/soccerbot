import enum

import config
from team import Team
from soccer_msgs.msg import GameState
from strategy.decision_tree_strategy import DecisionTreeStrategy, State, Action, Agent
from strategy.interfaces import Kick, Navigate, ObjectProximity
from robot import Robot

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
class FormationState(State, ObjectProximity):
    class Formation(enum.IntEnum):
        INITIAL = 0
        ATTACKING = 1
        MIDFIELD = 2
        DEFENSIVE = 3

    FormationMap = {
        Formation.INITIAL: config.FORMATIONS["initial"],
        Formation.ATTACKING: config.FORMATIONS["attack"],
        Formation.DEFENSIVE: config.FORMATIONS["defensive"],
        Formation.MIDFIELD: config.FORMATIONS["midfield"],
    }

    def __init__(self, friendly_team=None, opponent_team=None, game_state=None):
        super(FormationState, self).__init__(friendly_team, opponent_team, game_state)
        self.formation = FormationState.Formation.INITIAL

    def evaluateNextFormation(self) -> Formation:
        if self.friendly_team.average_ball_position != None:
            if self.friendly_team.average_ball_position[0] < -1:  # ball in opponents side
                return FormationState.Formation.ATTACKING
            elif self.friendly_team.average_ball_position[0] < 1:
                return FormationState.Formation.MIDFIELD
            else:
                return FormationState.Formation.DEFENSIVE
        else:
            return FormationState.Formation.DEFENSIVE


    def getLegalActions(self, currAgent: Agent):
        if currAgent == Agent.FRIENDLY:
            team = self.friendly_team
        else:
            team = self.opponent_team

        actions = [a.value for a in FormationAction.Action]
        actions_without_kick = [a.value for a in FormationAction.Action if a.value != FormationAction.Action.KICK]

        agent_actions = []
        for robot in team.robots:
            if self.can_kick_ball(robot, team.average_ball_position):
                agent_actions.append(actions)
            else:
                agent_actions.append(actions_without_kick)

        action_combinations = []
        for player1action in agent_actions[0]:
            for player2action in agent_actions[1]:
                for player3action in agent_actions[2]:
                    for player4action in agent_actions[3]:
                        action_combinations.append((player1action, player2action, player3action, player4action))
        return action_combinations


    def generateSuccessor(self, currAgent: Agent, move):
        s = State(self.friendly_team, self.opponent_team, self.game_state)
        # TODO determine next state based on current state
        return s

    def isWin(self):
        return False

    def isLose(self):
        return False

class FormationDecisionTreeStrategy(DecisionTreeStrategy, Kick, Navigate, ObjectProximity):

    def evaluationFunction(self, state) -> float:
        # Higher score if the ball is closer to the opponent net
        return state.friendly_team.average_ball_position[0]

    def executeBestMove(self, state: FormationState, action: FormationAction):
        def executeMove(robot, action: FormationAction.Action):
            if action == FormationAction.Action.STAND_STILL:
                pass
            elif action == FormationAction.Action.GO_TO_FORMATION_POSITION:
                self.navigation_to_position(robot, FormationState.FormationMap[state.formation][robot.role])
            elif action == FormationAction.Action.KICK:
                self.kick(robot, state.friendly_team.average_ball_position)
            elif action == FormationAction.Action.GO_TO_BALL:
                self.navigation_to_position(robot, state.friendly_team.average_ball_position.position)

        executeMove(state.friendly_team.robots[0], action.player_1_action)
        executeMove(state.friendly_team.robots[1], action.player_2_action)
        executeMove(state.friendly_team.robots[2], action.player_3_action)
        executeMove(state.friendly_team.robots[3], action.player_4_action)

    def getInitialState(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        return FormationState(friendly_team, opponent_team, game_state)