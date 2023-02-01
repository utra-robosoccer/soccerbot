import os
from typing import Union

import rospy

from soccer_msgs.msg import GameState
from soccer_strategy.ball import Ball
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled_3d import RobotControlled3D
from soccer_strategy.robot_observed import RobotObserved
from soccer_strategy.strategy.strategy_determine_side import StrategyDetermineSide
from soccer_strategy.strategy.strategy_dummy import StrategyDummy
from soccer_strategy.strategy.strategy_finished import StrategyFinished
from soccer_strategy.strategy.strategy_freekick import StrategyFreekick
from soccer_strategy.strategy.strategy_penaltykick import StrategyPenaltykick
from soccer_strategy.strategy.strategy_ready import StrategyReady
from soccer_strategy.strategy.strategy_set import StrategySet
from soccer_strategy.team import Team


class GameEngine3D:
    """
    Main game engine used by the actual robot and for webot's simulation
    """

    GAMESTATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("GAMESTATE")}
    SECONDARY_STATE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("STATE")}
    SECONDARY_STATE_MODE_LOOKUP = {getattr(GameState, key): key for key in dir(GameState) if key.startswith("MODE")}

    def __init__(self):
        """
        Initializes the game engine information
        """
        robot_id = rospy.get_param("~robot_id", 1)
        team_id = int(os.getenv("ROBOCUP_TEAM_ID", "16"))
        rospy.loginfo(f"Initializing strategy with robot id: {robot_id},  team id:  {team_id}")

        robots: [Union[RobotObserved, RobotControlled3D]] = []
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

    @property
    def this_robot(self) -> RobotControlled3D:
        """
        Returns this robot, of type RobotControlled3D
        :return: this robot, of type RobotControlled3D
        """
        return self.team1.robots[int(os.getenv("ROBOCUP_ROBOT_ID", 1)) - 1]

    def gamestate_callback(self, gameState: GameState):
        """
        Callback function for gamestate information from the webot's game controller

        :param gameState: Contains information about the game, whether it is playing or paused etc
        """
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
            self.this_robot.status = Robot.Status.PENALIZED
            self.this_robot.update_robot_state(None)  # immediately to stop actuators

    def decide_strategy(self):
        """
        Take information from the gamestate obtained by the game controller and decide what strategy based on the game controller
        Etc: If the game just started, it is in initial state, then the robot is in determining side.

        :return:
        """

        new_strategy = StrategyDetermineSide
        current_strategy = type(self.team1.strategy)

        if self.gameState.gameState == GameState.GAMESTATE_INITIAL:
            new_strategy = StrategyDetermineSide
        elif self.gameState.gameState == GameState.GAMESTATE_READY:
            if self.this_robot.status in [
                Robot.Status.DISCONNECTED,
                Robot.Status.PENALIZED,
            ]:
                new_strategy = StrategyDetermineSide
            elif current_strategy is StrategyDetermineSide and not self.team1.strategy.complete:
                new_strategy = StrategyDetermineSide
            else:
                if rospy.get_param("skip_ready_strategy", False):
                    new_strategy = StrategyDummy
                else:
                    new_strategy = StrategyReady
        elif self.gameState.gameState == GameState.GAMESTATE_SET:
            if self.this_robot.status == Robot.Status.PENALIZED:
                new_strategy = StrategyDetermineSide
            else:
                new_strategy = StrategySet
        elif self.gameState.gameState == GameState.GAMESTATE_FINISHED:
            new_strategy = StrategyFinished
        elif self.gameState.gameState == GameState.GAMESTATE_PLAYING:
            if self.gameState.secondaryState == GameState.STATE_NORMAL:
                if self.this_robot.status in [Robot.Status.PENALIZED, Robot.Status.DETERMINING_SIDE]:
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
        """
        Main loop for the strategy execution
        """

        while not rospy.is_shutdown():

            # Decide what strategy to run
            self.decide_strategy()

            # Log information about the strategy and run the strategy in the step_strategy function
            penalize_str = f"(P{self.gameState.penalty} - {self.gameState.secondsTillUnpenalized})" if self.gameState.penalty != 0 else ""
            print(
                f"\033[1mRobot {os.getenv('ROBOCUP_ROBOT_ID', 1)} Running {str(type(self.team1.strategy))} ({self.team1.strategy.iteration}) | Game State: {GameEngine3D.GAMESTATE_LOOKUP[self.gameState.gameState]}, Secondary State: {GameEngine3D.SECONDARY_STATE_LOOKUP[self.gameState.secondaryState]}, Secondary State Mode: {GameEngine3D.SECONDARY_STATE_MODE_LOOKUP[self.gameState.secondaryStateMode]} {penalize_str} [{rospy.Time.now().secs}.{rospy.Time.now().nsecs}]\033[0m"
            )
            self.team1.log()
            self.team1.strategy.step_strategy(self.team1, self.team2, self.gameState)

            # Sleep for a determined period of time decided by the strategy
            rospy.sleep(self.team1.strategy.update_frequency)
