import os
import sys
from importlib import reload
from os.path import exists
from types import ModuleType

import numpy as np
import pybullet as pb
import pytest
import yaml

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

from soccer_common.transformation import Transformation

run_in_ros = False
display = False
TEST_TIMEOUT = 60

if run_in_ros:
    import rospy

    rospy.init_node("soccer_control")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_pycontrol'")
    os.system("/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_trajectories'")

else:
    sys.modules["rospy"] = MagicMock()
    sys.modules["soccer_msgs"] = __import__("soccer_msgs_mock")
    import rospy

    rospy.Time = MagicMock()
    joint_state = MagicMock()
    joint_state.position = [0.0] * 18
    rospy.wait_for_message = MagicMock(return_value=joint_state)
    rospy.loginfo_throttle = lambda a, b: None

robot_model = "bez1"


def f(a, b):
    a = a.lstrip("~")
    if a == "robot_model":
        return robot_model

    config_path = f"../../config/{robot_model}_sim.yaml"
    if not exists(config_path):
        return b

    with open(config_path, "r") as g:

        y = yaml.safe_load(g)
        for c in a.split("/"):
            if y is None or c not in y:
                return b
            y = y[c]
        return y


rospy.get_param = f
import soccer_pycontrol.soccerbot_controller
from soccer_pycontrol.soccerbot import Links
from soccer_pycontrol.soccerbot_controller import SoccerbotController
from soccer_pycontrol.soccerbot_controller_ros import SoccerbotControllerRos


class TestWalking:
    robot_models = ["bez1", "bez3"]

    @staticmethod
    @pytest.fixture(params=robot_models)
    def walker(request):
        global robot_model
        robot_model = request.param

        if run_in_ros:
            c = SoccerbotControllerRos()
        else:
            for i in range(2):
                for attribute_name in dir(soccer_pycontrol):
                    if attribute_name == __name__.replace(__package__ + ".", ""):
                        continue
                    attribute = getattr(soccer_pycontrol, attribute_name)
                    if type(attribute) is ModuleType:
                        reload(attribute)
            c = SoccerbotController(display=display)
        yield c
        del c

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_ik(self, walker: SoccerbotController):
        walker.soccerbot.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsRightFoot(
            np.copy(walker.soccerbot.right_foot_init_position)
        )
        walker.soccerbot.configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsLeftFoot(
            np.copy(walker.soccerbot.left_foot_init_position)
        )

        pb.setJointMotorControlArray(
            bodyIndex=walker.soccerbot.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 20, 1)),
            targetPositions=walker.soccerbot.get_angles(),
        )
        for _ in range(100):
            if _ % 16 == 0:
                _ = _
            pb.stepSimulation()

    def test_walk_1(self, walker: SoccerbotController):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        walker.setGoal(Transformation([1, 0, 0], [0, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_2(self, walker: SoccerbotController):
        walker.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_3(self, walker: SoccerbotController):
        walker.setPose(Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_4(self, walker: SoccerbotController):
        walker.setPose(Transformation([0.3275415, 0.2841, 0.321], [0.04060593, 0.0120126, 0.86708929, -0.4963497]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_5(self, walker: SoccerbotController):
        walker.setPose(Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_6(self, walker: SoccerbotController):
        walker.setPose(Transformation([2.008, -0.646, 0.0], [0.0149, -0.0474, 0.99985, -0.0072]))
        walker.wait(100)
        walker.ready()
        walker.setGoal(Transformation([0.00736, 0.0356, 0.0], [0, 0, 0.998, 0.0176]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_walk_7(self, walker: SoccerbotController):
        walker.setPose(
            Transformation(
                [2.082603318747387, 0.04499586647232634, 0.0],
                [0.07888602209666294, -0.03018659995378454, 0.9054426772657052, 0.41597995490997813],
            )
        )
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([2.5901226468203067, 0.7938447967981127, 0.0], [0, 0, -0.9987013856398979, 0.050946465244882694]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=2)
    def test_walk_side(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0, -1, 0], [0.00000, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=2)
    def test_walk_backward(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([-1, 0.3, 0], [0.00000, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_turn_in_place(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)

        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_0(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi / 5, 0, 0])
        goal.set_position([0.05, 0.05, 0])
        walker.setGoal(goal)
        # walker.soccerbot.robot_path.show()
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_1(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([0.15, 0.05, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_2(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation.get_transform_from_euler([np.pi, 0, 0])
        goal.set_position([-0.3, 0, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_3(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation.get_transform_from_euler([-np.pi / 2, 0, 0])
        goal.set_position([-0.2, -0.2, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_4(self, walker: SoccerbotController):
        walker.setPose(Transformation([0.2489, -0.163, 0.0], [0.0284, -0.003, 0.9939, 0.01986]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0503, 0.06323, 0], [0, 0, 1, 0]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_small_movement_5(self, walker: SoccerbotController):
        walker.setPose(
            Transformation(
                [0.3096807057334623, 0.09374110438873018, 0.0],
                [0.03189331238935847, -0.0065516868290173, 0.9990119776602083, 0.03024831426656374],
            )
        )
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.14076394628045208, -0.034574636811865296, 0], [0, 0, -0.9999956132297835, -0.002962013029887055]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(TEST_TIMEOUT)
    @pytest.mark.flaky(reruns=1)
    def test_do_nothing(self, walker: SoccerbotController):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation.get_transform_from_euler([0, 0, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success
