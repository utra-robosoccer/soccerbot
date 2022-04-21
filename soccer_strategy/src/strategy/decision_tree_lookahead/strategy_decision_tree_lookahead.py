import abc
import enum
from copy import deepcopy

from strategy.strategy import Strategy
from team import Team

from soccer_msgs.msg import GameState


class Agent(enum.IntEnum):
    FRIENDLY = 0
    OPPONENT = 1


class Action:
    def __init__(self):
        self.player_1_action = None
        self.player_2_action = None
        self.player_3_action = None
        self.player_4_action = None


# State
class State:
    def __init__(self, friendly_team: Team = None, opponent_team: Team = None, game_state: GameState = None):
        self.friendly_team = deepcopy(friendly_team)
        self.opponent_team = deepcopy(opponent_team)
        self.game_state = deepcopy(game_state)

    @abc.abstractmethod
    def getLegalActions(self, currAgent: Agent) -> []:
        raise NotImplementedError

    @abc.abstractmethod
    def generateSuccessor(self, currAgent: Agent, move: Action):
        raise NotImplementedError

    @abc.abstractmethod
    def isWin(self):
        return False

    @abc.abstractmethod
    def isLose(self):
        return False


class StrategyDecisionTreeLookhead(Strategy):
    def __init__(self):
        super().__init__()
        self.depth = self.TREE_DEPTH  # Max depth traversal

    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)

        state = self.getInitialState(friendly_team, opponent_team, game_state)
        best_move, value = self.DECISION_ALGORITHM(state, Agent.FRIENDLY, 0)
        self.executeBestMove(state, best_move)

    @abc.abstractmethod
    def getInitialState(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        raise NotImplementedError

    @abc.abstractmethod
    def executeBestMove(self, state: State, action: Action):
        raise NotImplementedError

    def evaluationFunction(self, state) -> float:
        # Higher score if the ball is closer to the opponent net
        return state.friendly_team.average_ball_position[0]

    # Decision Tree Algorithms
    @abc.abstractmethod
    def PredeterminedDecisionTree(self, state: State, currAgent: Agent, depth: int) -> (Action, float):
        raise NotImplementedError

    def DFMiniMax(self, state: State, currAgent: Agent, depth: int) -> (Action, float):
        best_move = None
        if depth >= self.depth * 2 or state.isWin() or state.isLose():
            return best_move, self.evaluationFunction(state)
        if currAgent == Agent.FRIENDLY:
            value = float("-inf")
            nextAgent = Agent.OPPONENT
        else:
            value = float("inf")
            nextAgent = Agent.FRIENDLY

        for move in state.getLegalActions(currAgent):
            next_pos = state.generateSuccessor(currAgent, move)

            next_move, nxt_val = self.DFMiniMax(next_pos, nextAgent, depth + 1)
            if currAgent == 0:
                if value < nxt_val:
                    value, best_move = nxt_val, move
            else:
                if value > nxt_val:
                    value, best_move = nxt_val, move
        return best_move, value

    DECISION_ALGORITHM = DFMiniMax
    TREE_DEPTH = 4
