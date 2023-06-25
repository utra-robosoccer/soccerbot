import os
import threading

from soccer_common import Transformation
from soccer_strategy.communication.game_controller_receiver import GameStateReceiver
from soccer_strategy.game_engine_2d_with_referee import GameEngine2DWithReferee
from soccer_strategy.referee_2d import Referee2D
from soccer_strategy.strategy.strategy_determine_side import StrategyDetermineSide

os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest import TestCase
from unittest.mock import MagicMock, patch

import numpy as np
import rospy

from soccer_strategy.game_engine_2d import GameEngine2D
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled_2d import RobotControlled2D
from soccer_strategy.strategy.strategy_dummy import StrategyDummy
from soccer_strategy.strategy.strategy_stationary import StrategyStationary
from soccer_strategy.strategy.utils import Utility
from soccer_strategy.team import Team


class TestGameEngine2D(TestCase):
    def setUp(self) -> None:

        super().setUpClass()
        self.display = True
        if "DISPLAY" not in os.environ:
            self.display = False

    def test_determine_side(self):
        rospy.init_node("test")

        FIELD_LENGTH = 9
        FIELD_WIDTH = 3.15
        GOAL_WIDTH = 2.6

        d = StrategyDetermineSide()
        current_robot = MagicMock()
        game_state = MagicMock()

        # bottom right position, close post detected
        current_robot.robot_id = 1
        footprint_to_goal_post = Transformation(position=[FIELD_WIDTH - GOAL_WIDTH / 2, -FIELD_LENGTH / 2 + 1, 0])
        d.determine_side(current_robot, footprint_to_goal_post, game_state)

        # bottom left position, far post detected
        current_robot.robot_id = 2
        footprint_to_goal_post = Transformation(position=[-(FIELD_WIDTH + GOAL_WIDTH / 2 + 0.05), -FIELD_LENGTH / 2 + 4, 0])
        d.determine_side(current_robot, footprint_to_goal_post, game_state)

        # bottom right position, distant close post detected
        current_robot.robot_id = 1
        footprint_to_goal_post = Transformation(position=[-(FIELD_WIDTH - GOAL_WIDTH / 2), FIELD_LENGTH / 2 + 1, 0])
        d.determine_side(current_robot, footprint_to_goal_post, game_state)

        # top right position, close post detected
        current_robot.robot_id = 3
        footprint_to_goal_post = Transformation(position=[-(FIELD_WIDTH - GOAL_WIDTH / 2 - 0.05), FIELD_LENGTH / 2 - 1, 0])
        d.determine_side(current_robot, footprint_to_goal_post, game_state)

        # top right position, far post detected
        current_robot.robot_id = 3
        footprint_to_goal_post = Transformation(position=[-(FIELD_WIDTH - GOAL_WIDTH / 2 - 0.05), -FIELD_LENGTH / 2 - 1, 0])
        d.determine_side(current_robot, footprint_to_goal_post, game_state)

    def test_dummy_vs_stationary_strategy(self):
        rospy.init_node("test")

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, game_duration=4)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_dummy_vs_dummy_strategy(self):
        rospy.init_node("test")

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy, game_duration=6)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_obstacle_detection_between_ball_and_goal(self):
        rospy.init_node("test")
        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-1.5, 0, 0]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([1, 0, -3.14]),
                )
            ]
        )

        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_bump_into_ball_when_turning(self):
        rospy.init_node("test")
        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([0.07, 0.05, -3.14]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([1, 0, -3.14]),
                )
            ]
        )
        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_obstacle_detection_between_player_and_ball(self):
        rospy.init_node("test")

        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-1.5, -1.5, 0]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([-1, -1, -3.14]),
                ),
                RobotControlled2D(
                    robot_id=3,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([-0.4, -0.75, -3.14]),
                ),
            ]
        )

        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_search_for_ball(self):
        rospy.init_node("test")

        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-3, 0, 0]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([3, 0, -3.14]),
                )
            ]
        )

        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_one_v_one(self):
        rospy.init_node("test")

        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([-2, 0, 0]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([1, 0, -3.14]),
                )
            ]
        )

        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_obstacle_detection_when_ball_behind_kicker(self):
        rospy.init_node("test")

        team1 = Team(
            [
                RobotControlled2D(
                    robot_id=1,
                    team=Robot.Team.FRIENDLY,
                    role=Robot.Role.STRIKER,
                    status=Robot.Status.READY,
                    position=np.array([1, 0, -3.14]),
                )
            ]
        )

        team2 = Team(
            [
                RobotControlled2D(
                    robot_id=2,
                    team=Robot.Team.OPPONENT,
                    role=Robot.Role.GOALIE,
                    status=Robot.Status.READY,
                    position=np.array([4, 0, -3.14]),
                )
            ]
        )

        g = GameEngine2D(
            display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, team_1=team1, team_2=team2, game_duration=4
        )

        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_navigate_to_scoring_position_with_offset_case_1(self):

        robot = MagicMock()
        goal_position = np.array([5, 0])

        robot.position = goal_position - np.array([0.4, 0.2])
        ball_position = goal_position - np.array([0.2, 0.4])
        navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -0.6]))
        Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    def test_navigate_to_scoring_position_with_offset_case_2a(self):

        robot = MagicMock()
        goal_position = np.array([5, 0])

        robot.position = goal_position - np.array([1, 1.0])
        ball_position = goal_position - np.array([0.4, 1.2])
        navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))
        Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    def test_navigate_to_scoring_position_with_offset_case_2b(self):

        robot = MagicMock()
        goal_position = np.array([5, 0])

        robot.position = goal_position - np.array([0.4, 1.0])
        ball_position = goal_position - np.array([0.4, 1.2])
        navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))
        assert Utility.navigate_to_position_with_offset.call_args.kwargs["offset"] == 0.3
        Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    def test_navigate_to_scoring_position_with_offset_case_3_out_of_the_edge(self):

        robot = MagicMock()
        goal_position = np.array([5, 0])

        robot.position = goal_position - np.array([0.5, 1.0])
        ball_position = goal_position - np.array([0.4, 1.4])
        navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], goal_position)
        Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    def test_navigate_to_scoring_position_with_offset_case_3_regular(self):

        robot = MagicMock()
        goal_position = np.array([5, 0])

        robot.position = goal_position - np.array([1.0, 1.0])
        ball_position = goal_position - np.array([0.8, 0.5])
        navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], goal_position)
        Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    def test_navigate_to_scoring_position_no_crash(self):

        robot = MagicMock()
        for goal_position in [np.array([5, 0]), np.array([-5, 0])]:
            for robot_x in np.linspace(-6, 6, 5):
                for robot_y in np.linspace(-3, 3, 5):
                    for ball_x in np.linspace(-6, 6, 5):
                        for ball_y in np.linspace(-3, 3, 5):
                            robot.position = goal_position - np.array([robot_x, robot_y])
                            ball_position = goal_position - np.array([ball_x, ball_y])
                            navigate_to_position_with_offset_original = Utility.navigate_to_position_with_offset
                            Utility.navigate_to_position_with_offset = MagicMock()
                            Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
                            Utility.navigate_to_position_with_offset = navigate_to_position_with_offset_original

    @patch("referee.Supervisor")
    def test_2d_with_referee(self, referee):
        rospy.init_node("test")
        os.chdir("../../../external/hlvs_webots/controllers/referee")

        g = GameEngine2DWithReferee(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy, game_duration=6)

        # Referee
        referee2d = Referee2D(game_engine_2d=g)

        # Game Controller receiver
        publisher_init_orig = rospy.Publisher.__init__

        game_controller_receivers = {}
        game_controller_receivers_threads = {}
        for team in [16, 5]:
            os.environ["ROBOCUP_TEAM_ID"] = str(team)
            for player in [1, 2, 3, 4]:
                rospy.set_param("robot_id", player)
                os.environ["ROS_NAMESPACE"] = f"robot{player}"

                rospy.Publisher.__init__ = lambda self, name, *args, **kwargs: publisher_init_orig(
                    self, f"team_{team}/robot{player}/{name}", *args, **kwargs
                )
                game_controller_receivers[(team, player)] = GameStateReceiver()
                game_controller_receivers_threads[(team, player)] = threading.Thread(target=game_controller_receivers[(team, player)].receive_forever)
                game_controller_receivers_threads[(team, player)].start()

        # Referee thread
        referee_main_loop_thread = threading.Thread(target=referee2d.main_loop)
        referee_main_loop_thread.start()
        game_controller_receivers_threads["referee"] = referee_main_loop_thread

        # Game running
        g.run()

        for gcrt in game_controller_receivers_threads.values():
            gcrt.join()
        pass
