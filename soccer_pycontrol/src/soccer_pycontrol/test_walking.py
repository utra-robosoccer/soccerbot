import os
import sys
import time

import numpy as np
from timeout_decorator import timeout_decorator

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest import TestCase
from unittest.mock import MagicMock

from soccer_common.transformation import Transformation

run_in_ros = False
display = True
TEST_TIMEOUT = 60
if "pytest" in sys.argv[0]:
    run_in_ros = False
    display = False
else:
    import rospy

if run_in_ros:
    rospy.init_node("soccer_control")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_pycontrol'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_trajectories'")
    from soccer_pycontrol.soccerbot_controller_ros import SoccerbotControllerRos
else:
    sys.modules["rospy"] = MagicMock()
    sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
    import rospy

    rospy.Time = MagicMock()
    joint_state = MagicMock()
    joint_state.position = [0.0] * 18
    rospy.wait_for_message = MagicMock(return_value=joint_state)
    rospy.loginfo_throttle = lambda a, b: None
    rospy.get_param = lambda a, b: b
    from soccer_pycontrol.soccerbot_controller import SoccerbotController


class TestWalking(TestCase):
    def setUp(self) -> None:
        super().setUp()
        if run_in_ros:
            self.walker = SoccerbotControllerRos()
        else:
            self.walker = SoccerbotController(display=display)

    def tearDown(self) -> None:
        super().tearDown()
        del self.walker

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_1(self):
        self.walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_2(self):
        self.walker.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_3(self):
        self.walker.setPose(Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_4(self):
        self.walker.setPose(Transformation([0.3275415, 0.2841, 0.321], [0.04060593, 0.0120126, 0.86708929, -0.4963497]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_5(self):
        self.walker.setPose(Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_6(self):
        self.walker.setPose(Transformation([2.008, -0.646, 0.0], [0.0149, -0.0474, 0.99985, -0.0072]))
        self.walker.wait(100)
        self.walker.ready()
        self.walker.setGoal(Transformation([0.00736, 0.0356, 0.0], [0, 0, 0.998, 0.0176]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_7(self):
        self.walker.setPose(
            Transformation(
                [2.082603318747387, 0.04499586647232634, 0.0],
                [0.07888602209666294, -0.03018659995378454, 0.9054426772657052, 0.41597995490997813],
            )
        )
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([2.5901226468203067, 0.7938447967981127, 0.0], [0, 0, -0.9987013856398979, 0.050946465244882694]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_side(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0, -1, 0], [0.00000, 0, 0, 1]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_walk_backward(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([-1, 0.3, 0], [0.00000, 0, 0, 1]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_turn_in_place(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)

        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        self.walker.setGoal(goal)
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_0(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi / 5, 0, 0])
        goal.set_position([0.05, 0.05, 0])
        self.walker.setGoal(goal)
        # self.walker.soccerbot.robot_path.show()
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_1(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([0.15, 0.05, 0])
        self.walker.setGoal(goal)
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_2(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([-0.3, 0, 0])
        self.walker.setGoal(goal)
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_3(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([-np.pi / 2, 0, 0])
        goal.set_position([-0.2, -0.2, 0])
        self.walker.setGoal(goal)
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_4(self):
        self.walker.setPose(Transformation([0.2489, -0.163, 0.0], [0.0284, -0.003, 0.9939, 0.01986]))
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.0503, 0.06323, 0], [0, 0, 1, 0]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_small_movement_5(self):
        self.walker.setPose(
            Transformation(
                [0.3096807057334623, 0.09374110438873018, 0.0],
                [0.03189331238935847, -0.0065516868290173, 0.9990119776602083, 0.03024831426656374],
            )
        )
        self.walker.ready()
        self.walker.wait(100)
        self.walker.setGoal(Transformation([0.14076394628045208, -0.034574636811865296, 0], [0, 0, -0.9999956132297835, -0.002962013029887055]))
        walk_success = self.walker.run()
        self.assertTrue(walk_success)

    @timeout_decorator.timeout(TEST_TIMEOUT)
    def test_do_nothing(self):
        self.walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        self.walker.ready()
        self.walker.wait(100)
        goal = Transformation.get_transform_from_euler([0, 0, 0])
        self.walker.setGoal(goal)
        walk_success = self.walker.run()
        self.assertTrue(walk_success)
