import sys
import unittest
from unittest import TestCase
from unittest.mock import MagicMock

import numpy
import numpy as np

from soccer_common.mock_ros import mock_ros


class Test(TestCase):
    def setUp(self) -> None:

        mock_ros(robot_model="bez1", real_robot=False, config_path="")

        super().setUpClass()
        self.display = True
        if "pytest" in sys.argv[0]:
            self.display = False

    def test_corner_kick(self):
        sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
        from game_engine_2d import GameEngine2D
        from strategy.strategy_cornerkick import StrategyCornerkick
        from strategy.strategy_dummy import StrategyDummy
        from strategy.strategy_stationary import StrategyStationary

        mock_ros(robot_model="bez1", real_robot=False, config_path="")

        corners = [np.array([4.5, 3]), np.array([4.5, -3])]
        random_corner = corners[np.random.randint(0, 2)]

        g = GameEngine2D(
            display=self.display,
            team_1_strategy=StrategyCornerkick,
            team_2_strategy=StrategyStationary,
            game_duration=5,
            init_ball_position=random_corner,
        )
        friendly_points, opponent_points = g.run()

        # pass if the ball is near the target position
        assert np.allclose(StrategyCornerkick.CORNERKICK_TARGET, g.ball.position, atol=2)

    def test_dummy_vs_stationary_strategy(self):
        sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
        from game_engine_2d import GameEngine2D
        from strategy.strategy_dummy import StrategyDummy
        from strategy.strategy_stationary import StrategyStationary

        mock_ros(robot_model="bez1", real_robot=False, config_path="")

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, game_duration=2)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_dummy_vs_dummy_strategy(self):
        sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
        from game_engine_2d import GameEngine2D
        from strategy.strategy_dummy import StrategyDummy
        from strategy.strategy_stationary import StrategyStationary

        mock_ros(robot_model="bez1", real_robot=False, config_path="")

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, game_duration=2)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_navigate_to_scoring_position_with_offset_case_1(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.4, 0.2])
        ball_position = goal_position - numpy.array([0.2, 0.4])
        Actions.navigate_to_position_with_offset = MagicMock()
        Actions.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Actions.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][2], np.array([5, -0.6]))

    def test_navigate_to_scoring_position_with_offset_case_2a(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([1, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.2])
        Actions.navigate_to_position_with_offset = MagicMock()
        Actions.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Actions.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))

    def test_navigate_to_scoring_position_with_offset_case_2b(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.4, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.2])
        Actions.navigate_to_position_with_offset = MagicMock()
        Actions.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Actions.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))
        assert Actions.navigate_to_position_with_offset.call_args.kwargs["offset"] == 0.3

    def test_navigate_to_scoring_position_with_offset_case_3_out_of_the_edge(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.5, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.4])
        Actions.navigate_to_position_with_offset = MagicMock()
        Actions.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Actions.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][2], goal_position)

    def test_navigate_to_scoring_position_with_offset_case_3_regular(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([1.0, 1.0])
        ball_position = goal_position - numpy.array([0.8, 0.5])
        Actions.navigate_to_position_with_offset = MagicMock()
        Actions.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Actions.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Actions.navigate_to_position_with_offset.call_args[0][2], goal_position)

    def test_navigate_to_scoring_position_no_crash(self):
        from strategy.interfaces.actions import Actions

        robot = MagicMock()
        for goal_position in [numpy.array([5, 0]), numpy.array([-5, 0])]:
            for robot_x in np.linspace(-6, 6, 5):
                for robot_y in np.linspace(-3, 3, 5):
                    for ball_x in np.linspace(-6, 6, 5):
                        for ball_y in np.linspace(-3, 3, 5):
                            robot.position = goal_position - numpy.array([robot_x, robot_y])
                            ball_position = goal_position - numpy.array([ball_x, ball_y])
                            Actions.navigate_to_position_with_offset = MagicMock()
                            Actions.navigate_to_scoring_position(robot, ball_position, goal_position)

    @unittest.skip("Under Development")
    def test_formation_strategy(self):
        sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
        from game_engine_2d import GameEngine2D
        from strategy.decision_tree.strategy_decision_tree import StrategyDecisionTree
        from strategy.strategy_stationary import StrategyStationary

        mock_ros(robot_model="bez1", real_robot=False, config_path="")

        friendly_wins = 0
        opponent_wins = 0
        for i in range(1):
            g = GameEngine2D(display=self.display, team_1_strategy=StrategyDecisionTree, team_2_strategy=StrategyStationary)
            friendly_points, opponent_points = g.run()
            if friendly_points > opponent_points:
                friendly_wins += 1
            elif friendly_points < opponent_points:
                opponent_wins += 1
        print(f"Friendly: {friendly_wins}, opponent: {opponent_wins}")
