import os
import numpy as np
import tf
import rospy
from std_msgs.msg import Empty, Int32, Bool
from soccer_msgs.msg import GameState
from robot_controlled_3d import RobotControlled3D
from robot import Robot
from robot_observed import RobotObserved
from ball import Ball
from team import Team
import game_engine
from strategy.strategy_freekick import StrategyFreekick
from strategy.decision_tree_lookahead.strategy_decision_tree_lookahead import StrategyDecisionTreeLookhead
from strategy.strategy_penaltykick import StrategyPenaltykick
from strategy.strategy_ready import StrategyReady
from strategy.strategy_initial import StrategyInitial
from strategy.strategy_finished import StrategyFinished
from strategy.strategy_set import StrategySet


class GameEngine3D(game_engine.GameEngine):
    # Seconds
    STRATEGY_UPDATE_INTERVAL = 1
    TEAM_COMMUNICATION_LOG_INTERVAL = 5

    GameStateMap = ["GAMESTATE_INITIAL",
                    "GAMESTATE_READY",
                    "GAMESTATE_SET",
                    "GAMESTATE_PLAYING",
                    "GAMESTATE_FINISHED"]

    SecondaryStateModeMap = ["PREPARATION",
                             "PLACING",
                             "END"]

    SecondaryGameStateMap = ["STATE_NORMAL",
                             "STATE_PENALTYSHOOT",
                             "STATE_OVERTIME",
                             "STATE_TIMEOUT",
                             "STATE_DIRECT_FREEKICK",
                             "STATE_INDIRECT_FREEKICK",
                             "STATE_PENALTYKICK",
                             "STATE_CORNER_KICK",
                             "STATE_GOAL_KICK",
                             "STATE_THROW_IN"
                             ]


    def __init__(self):
        robot_id = os.getenv("ROBOCUP_ROBOT_ID", 1)
        robot_name = "robot" + str(robot_id)
        team_id = int(os.getenv("TEAM_ID", "16"))
        rospy.loginfo(f"Initializing strategy with robot id: {robot_id}, robot_name: {robot_name},  team id {team_id}")

        self.team1 = Team([
            RobotObserved(robot_id=1, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotObserved(robot_id=2, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotObserved(robot_id=3, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotObserved(robot_id=4, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY)
        ])
        self.team1.robots[robot_id] = RobotControlled3D(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name=robot_name)
        self.team2 = Team({})
        self.ball = Ball(position=None)

        # GameState
        self.gameState = GameState()
        self.game_state_subscriber = rospy.Subscriber('gamestate', GameState, self.gamestate_callback)

    def gamestate_callback(self, gameState: GameState):
        print("Game State Changed")
        print("Game State: " + GameEngine3D.GameStateMap[gameState.gameState])
        print("Secondary State: " + GameEngine3D.SecondaryStateModeMap[gameState.secondaryState])
        print("Secondary State Mode: " + GameEngine3D.SecondaryStateModeMap[gameState.secondaryStateMode])
        print(gameState)
        self.gameState = gameState

    def decide_strategy(self):
        if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
            self.team1.strategy = StrategyInitial()
        elif self.gameState.gameState == GameState.GAMESTATE_READY:
            self.team1.strategy = StrategyReady()
        elif self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                if self.gameState.hasKickOff:
                    self.team1.strategy = StrategyDecisionTreeLookhead()
                else:
                    self.team1.strategy = StrategySet()
            elif self.gameState.secondaryState == GameState.STATE_DIRECT_FREEKICK:
                self.team1.strategy = StrategyFreekick()
            elif self.gameState.secondaryState == GameState.STATE_INDIRECT_FREEKICK:
                self.team1.strategy = StrategyFreekick()
            elif self.gameState.secondaryState == GameState.STATE_CORNER_KICK:
                self.team1.strategy = StrategyFreekick()
            elif self.gameState.secondaryState == GameState.STATE_GOAL_KICK:
                self.team1.strategy = StrategyFreekick()
            elif self.gameState.secondaryState == GameState.STATE_THROW_IN:
                self.team1.strategy = StrategyFreekick()
            elif self.gameState.secondaryState == GameState.STATE_PENALTYKICK:
                self.team1.strategy = StrategyPenaltykick()
            elif self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                self.team1.strategy = StrategyPenaltykick()
        elif self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            self.team1.strategy = StrategyFinished()

    # run loop
    def run(self):
        rostime_previous = 0

        while not rospy.is_shutdown():
            rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

            if rostime % GameEngine3D.STRATEGY_UPDATE_INTERVAL < rostime_previous % GameEngine3D.STRATEGY_UPDATE_INTERVAL:

                # Decide what strategy to run
                self.decide_strategy()

                # Run the strategy
                self.team1.strategy.update_next_strategy(self.team1, self.team2, self.gameState)

            rostime_previous = rostime
