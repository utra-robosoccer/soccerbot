import os

import rosparam
import rospy
from ball import Ball
from robot import Robot
from robot_controlled_3d import RobotControlled3D
from robot_observed import RobotObserved
from strategy.strategy_determine_side import StrategyDetermineSide
from strategy.strategy_dummy import StrategyDummy
from strategy.strategy_finished import StrategyFinished
from strategy.strategy_freekick import StrategyFreekick
from strategy.strategy_penaltykick import StrategyPenaltykick
from strategy.strategy_ready import StrategyReady
from strategy.strategy_set import StrategySet
from team import Team

from soccer_msgs.msg import GameState


class GameEngine3D:
    GAMESTATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("GAMESTATE")}
    SECONDARY_STATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("STATE")}
    SECONDARY_STATE_MODE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("MODE")}

    def __init__(self):
        robot_id = rospy.get_param("~robot_id", 1)
        team_id = int(os.getenv("ROBOCUP_TEAM_ID", "16"))
        rospy.loginfo(f"Initializing strategy with robot id: {robot_id},  team id:  {team_id}")

        robots = []
        for i in range(1, 5):
            if i == robot_id:
                robots.append(RobotControlled3D(team=Robot.Team.FRIENDLY, role=Robot.Role.UNASSIGNED, status=Robot.Status.DISCONNECTED))
            else:
                robots.append(
                    RobotObserved(
                        robot_id=i,
                        team=Robot.Team.FRIENDLY,
                        role=Robot.Role.UNASSIGNED,
                        status=Robot.Status.DISCONNECTED,
                    )
                )
        self.team1 = Team(robots)
        self.team1.strategy = StrategyDetermineSide()
        self.team2 = Team([])
        self.team2.strategy = StrategyDetermineSide()
        self.ball = Ball()

        # GameState
        self.gameState = GameState()
        self.gameState.gameState = GameState.GAMESTATE_READY
        self.game_state_subscriber = rospy.Subscriber("gamestate", GameState, self.gamestate_callback)

    def robot(self) -> RobotControlled3D:
        return self.team1.robots[int(os.getenv("ROBOCUP_ROBOT_ID", 1)) - 1]

    def gamestate_callback(self, gameState: GameState):
        if (
            self.gameState.gameState != gameState.gameState
            or self.gameState.secondaryState != gameState.secondaryState
            or self.gameState.secondaryStateMode != gameState.secondaryStateMode
        ):
            print(
                f"\033[92mGame State Changed: {GameEngine3D.GAMESTATE_LOOKUP[gameState.gameState]}, Secondary State: {GameEngine3D.SECONDARY_STATE_LOOKUP[gameState.secondaryState]}, Secondary State Mode: {GameEngine3D.SECONDARY_STATE_MODE_LOOKUP[gameState.secondaryStateMode]}\033[0m"
            )

        self.gameState = gameState
        if self.gameState.penalty != GameState.PENALTY_NONE:
            self.robot().status = Robot.Status.PENALIZED
            self.robot().update_robot_state(None)  # Publish immediately to stop actuators

    def decide_strategy(self):
        new_strategy = StrategyDetermineSide
        current_strategy = type(self.team1.strategy)

        if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
            new_strategy = StrategyDetermineSide
        elif self.gameState.gameState == GameState.GAMESTATE_READY:
            if self.robot().status in [
                Robot.Status.DETERMINING_SIDE,
                Robot.Status.DISCONNECTED,
                Robot.Status.PENALIZED,
            ]:
                new_strategy = StrategyDetermineSide
            else:
                new_strategy = StrategyReady
        elif self.gameState.gameState == GameState.GAMESTATE_SET:
            if self.robot().status == Robot.Status.PENALIZED:
                new_strategy = StrategyDetermineSide
            else:
                new_strategy = StrategySet
        elif self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            new_strategy = StrategyFinished
        elif self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                if self.robot().status in [Robot.Status.PENALIZED, Robot.Status.DETERMINING_SIDE]:
                    new_strategy = StrategyDetermineSide
                elif current_strategy == StrategyDetermineSide and self.team1.strategy.complete:
                    new_strategy = StrategyReady
                elif current_strategy == StrategyReady and self.team1.strategy.complete:
                    new_strategy = StrategyDummy
                elif self.gameState.hasKickOff:
                    new_strategy = StrategyDummy
                else:
                    new_strategy = StrategyDummy
            elif self.gameState.secondaryState == GameState.STATE_DIRECT_FREEKICK:
                new_strategy = StrategyFreekick
            elif self.gameState.secondaryState == GameState.STATE_INDIRECT_FREEKICK:
                new_strategy = StrategyFreekick
            elif self.gameState.secondaryState == GameState.STATE_CORNER_KICK:
                new_strategy = StrategyFreekick
            elif self.gameState.secondaryState == GameState.STATE_GOAL_KICK:
                new_strategy = StrategyFreekick
            elif self.gameState.secondaryState == GameState.STATE_THROW_IN:
                new_strategy = StrategyFreekick
            elif self.gameState.secondaryState == GameState.STATE_PENALTYKICK:
                new_strategy = StrategyPenaltykick
            elif self.gameState.secondaryState == GameState.STATE_PENALTYSHOOT:
                new_strategy = StrategyPenaltykick
        else:
            raise Exception("No strategy covered by the current states")

        if type(self.team1.strategy) != new_strategy:
            self.team1.strategy = new_strategy()

    # run loop
    def run(self):
        while not rospy.is_shutdown():

            # Decide what strategy to run
            self.decide_strategy()
            # Run the strategy
            penalize_str = f"(P{self.gameState.penalty} - {self.gameState.secondsTillUnpenalized})" if self.gameState.penalty != 0 else ""
            print(
                f"\033[1mRobot {os.getenv('ROBOCUP_ROBOT_ID', 1)} Running {str(type(self.team1.strategy))} ({self.team1.strategy.iteration}) | Game State: {GameEngine3D.GAMESTATE_LOOKUP[self.gameState.gameState]}, Secondary State: {GameEngine3D.SECONDARY_STATE_LOOKUP[self.gameState.secondaryState]}, Secondary State Mode: {GameEngine3D.SECONDARY_STATE_MODE_LOOKUP[self.gameState.secondaryStateMode]} {penalize_str}\033[0m"
            )
            self.team1.log()
            self.team1.strategy.update_next_strategy(self.team1, self.team2, self.gameState)

            rospy.sleep(self.team1.strategy.update_frequency)
