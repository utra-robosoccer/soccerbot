import os
import numpy as np
import tf
import rospy
from std_msgs.msg import Empty, Int32, Bool
from soccer_msgs.msg import GameState
from robot_3d import Robot3D
from robot import Robot, RobotTeamMate
from ball import Ball
from team import Team
import game_engine
import config
from strategy.freekick_strategy import FreekickStrategy
from strategy.decision_tree.formation_strategy import FormationDecisionTreeStrategy
from strategy.penaltykick_strategy import PenaltykickStrategy
from strategy.ready_strategy import ReadyStrategy
from strategy.initial_strategy import InitialStrategy
from strategy.finished_strategy import FinishedStrategy
from strategy.set_strategy import SetStrategy


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
        robot_name = os.getenv("ROBOT_NAME", "robot1")
        team_id = int(os.getenv("TEAM_ID", "16"))
        rospy.loginfo(f"Initializing strategy with robot id: {robot_id}, robot_name: {robot_name},  team id {team_id}")

        self.team1 = Team([
            RobotTeamMate(robot_id=1, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotTeamMate(robot_id=2, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotTeamMate(robot_id=3, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY),
            RobotTeamMate(robot_id=4, team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.READY)
        ])
        self.team1.robots[robot_id] = Robot3D(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name=robot_name)
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
            self.team1.strategy = InitialStrategy()
        elif self.gameState.gameState == GameState.GAMESTATE_READY:
            self.team1.strategy = ReadyStrategy()
        elif self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                if self.gameState.hasKickOff:
                    self.team1.strategy = FormationDecisionTreeStrategy()
                else:
                    self.team1.strategy = SetStrategy()
            elif self.gameState.secondaryState == GameState.STATE_DIRECT_FREEKICK:
                self.team1.strategy = FreekickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_INDIRECT_FREEKICK:
                self.team1.strategy = FreekickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_CORNER_KICK:
                self.team1.strategy = FreekickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_GOAL_KICK:
                self.team1.strategy = FreekickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_THROW_IN:
                self.team1.strategy = FreekickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_PENALTYKICK:
                self.team1.strategy = PenaltykickStrategy()
            elif self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                self.team1.strategy = PenaltykickStrategy()
        elif self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            self.team1.strategy = FinishedStrategy()

    # run loop
    def run(self):
        rostime_previous = 0

        while not rospy.is_shutdown():
            rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

            if rostime % GameEngine3D.STRATEGY_UPDATE_INTERVAL < rostime_previous % GameEngine3D.STRATEGY_UPDATE_INTERVAL:

                # Decide what strategy to run
                self.decide_strategy()

                # Run the strategy
                self.team1.strategy.update_next_strategy(self.team1, self.team2, self.ball, self.gameState)

            rostime_previous = rostime
