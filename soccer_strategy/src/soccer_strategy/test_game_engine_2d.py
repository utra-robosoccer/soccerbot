import os

os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest import TestCase
from unittest.mock import MagicMock

import numpy
import numpy as np
import rospy
from vispy import app, scene


class TestGameEngine2D(TestCase):
    def setUp(self) -> None:

        super().setUpClass()
        self.display = True
        if "DISPLAY" not in os.environ:
            self.display = False

    def test_dummy_vs_stationary_strategy(self):
        rospy.init_node("test")

        from soccer_strategy.game_engine_2d import GameEngine2D
        from soccer_strategy.strategy.strategy_dummy import StrategyDummy
        from soccer_strategy.strategy.strategy_stationary import StrategyStationary

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyStationary, game_duration=2)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")
        assert not (friendly_points == 0 and opponent_points == 0)

    def test_dummy_vs_dummy_strategy(self):
        rospy.init_node("test")

        from soccer_strategy.game_engine_2d import GameEngine2D
        from soccer_strategy.strategy.strategy_dummy import StrategyDummy

        g = GameEngine2D(display=self.display, team_1_strategy=StrategyDummy, team_2_strategy=StrategyDummy, game_duration=2)
        friendly_points, opponent_points = g.run()
        print(f"Friendly: {friendly_points}, opponent: {opponent_points}")

    def test_navigate_to_scoring_position_with_offset_case_1(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.4, 0.2])
        ball_position = goal_position - numpy.array([0.2, 0.4])
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -0.6]))

    def test_navigate_to_scoring_position_with_offset_case_2a(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([1, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.2])
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))

    def test_navigate_to_scoring_position_with_offset_case_2b(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.4, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.2])
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], np.array([5, -1.2]))
        assert Utility.navigate_to_position_with_offset.call_args.kwargs["offset"] == 0.3

    def test_navigate_to_scoring_position_with_offset_case_3_out_of_the_edge(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([0.5, 1.0])
        ball_position = goal_position - numpy.array([0.4, 1.4])
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], goal_position)

    def test_navigate_to_scoring_position_with_offset_case_3_regular(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        goal_position = numpy.array([5, 0])

        robot.position = goal_position - numpy.array([1.0, 1.0])
        ball_position = goal_position - numpy.array([0.8, 0.5])
        Utility.navigate_to_position_with_offset = MagicMock()
        Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
        assert Utility.navigate_to_position_with_offset.call_args[0][0] == robot
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][1], ball_position)
        assert np.array_equal(Utility.navigate_to_position_with_offset.call_args[0][2], goal_position)

    def test_navigate_to_scoring_position_no_crash(self):
        from soccer_strategy.strategy.utils import Utility

        robot = MagicMock()
        for goal_position in [numpy.array([5, 0]), numpy.array([-5, 0])]:
            for robot_x in np.linspace(-6, 6, 5):
                for robot_y in np.linspace(-3, 3, 5):
                    for ball_x in np.linspace(-6, 6, 5):
                        for ball_y in np.linspace(-3, 3, 5):
                            robot.position = goal_position - numpy.array([robot_x, robot_y])
                            ball_position = goal_position - numpy.array([ball_x, ball_y])
                            Utility.navigate_to_position_with_offset = MagicMock()
                            Utility.navigate_to_scoring_position(robot, ball_position, goal_position)
