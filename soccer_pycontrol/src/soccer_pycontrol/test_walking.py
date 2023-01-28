import os

os.environ["ROS_NAMESPACE"] = "/robot1"

from unittest.mock import MagicMock

import numpy as np
import pybullet as pb
import pytest
import rospy
from matplotlib import pyplot as plt

from soccer_common.transformation import Transformation
from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_pycontrol.calibration import adjust_navigation_transform
from soccer_pycontrol.links import Links
from soccer_pycontrol.navigator import Navigator
from soccer_pycontrol.navigator_ros import NavigatorRos

joint_state = MagicMock()
joint_state.position = [0.0] * 18
rospy.wait_for_message = MagicMock(return_value=joint_state)


class TestWalking:
    @staticmethod
    @pytest.fixture
    def walker(request) -> Navigator:
        robot_model = request.param

        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../config/"
        config_path = config_folder_path + f"{robot_model}_sim_pybullet.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)
        if "DISPLAY" not in os.environ:
            c = Navigator(display=False, real_time=True)
        else:
            c = Navigator(display=True, real_time=True)

        yield c
        c.close()

    @staticmethod
    @pytest.fixture
    def walker_ros(request) -> NavigatorRos:
        robot_model = request.param

        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../config/"
        config_path = config_folder_path + f"{robot_model}_sim.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)

        c = NavigatorRos()

        yield c
        del c

    @staticmethod
    @pytest.fixture
    def walker_real_robot(request) -> NavigatorRos:
        robot_model = request.param

        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../config/"
        config_path = config_folder_path + f"{robot_model}.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)

        c = NavigatorRos()
        yield c
        del c

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1", "bez2", "bez3"], indirect=True)
    def test_ik(self, walker: Navigator):
        walker.soccerbot.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsRightFoot(
            np.copy(walker.soccerbot.right_foot_init_position)
        )
        walker.soccerbot.configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsLeftFoot(
            np.copy(walker.soccerbot.left_foot_init_position)
        )

        pb.setJointMotorControlArray(
            bodyIndex=walker.soccerbot.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=walker.soccerbot.get_angles(),
        )
        for _ in range(100):
            if _ % 16 == 0:
                _ = _
            pb.stepSimulation()

    @pytest.mark.parametrize("walker", ["bez1", "bez2", "bez3"], indirect=True)
    def test_walk_1(self, walker: Navigator):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        goal_position = Transformation([1, 0, 0], [0, 0, 0, 1])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

        final_position = walker.getPose()
        distance_offset = np.linalg.norm((final_position - goal_position.position)[0:2])
        assert distance_offset < 0.12

    @pytest.mark.skip
    def test_walk_1_ros(self, walker_ros: NavigatorRos):
        walker_ros.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker_ros.ready()
        walker_ros.wait(200)
        goal_position = Transformation([1, 0, 0], [0, 0, 0, 1])
        walker_ros.setGoal(goal_position)
        walk_success = walker_ros.run(single_trajectory=True)
        assert walk_success

        final_position = walker_ros.getPose()
        distance_offset = np.linalg.norm((final_position - goal_position.position)[0:2])
        print(f"Final distance offset {distance_offset}")

    @pytest.mark.skip
    def test_walk_1_real_robot(self, walker_ros: NavigatorRos):
        walker_ros.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker_ros.ready()
        walker_ros.wait(200)
        goal_position = Transformation([1, 0, 0], [0, 0, 0, 1])
        walker_ros.setGoal(goal_position)
        walk_success = walker_ros.run(single_trajectory=True)
        assert walk_success

        final_position = walker_ros.getPose()
        distance_offset = np.linalg.norm((final_position - goal_position.position)[0:2])

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_2(self, walker: Navigator):
        walker.setPose(Transformation([-0.7384, -0.008, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0198, -0.0199, 0], [0.00000, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_3(self, walker: Navigator):
        walker.setPose(Transformation([-2.404, -1.0135, 0], [0, 0, -0.9979391070307153, 0.064168050139]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([-2.26, -1.27, 0], [0, 0, 0.997836202477347, 0.06574886330262358]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_4(self, walker: Navigator):
        walker.setPose(Transformation([0.3275415, 0.2841, 0.321], [0.04060593, 0.0120126, 0.86708929, -0.4963497]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([-0.12015226, -0.19813691, 0.321], [0, 0, 0.95993011, -0.28023953]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_5(self, walker: Navigator):
        walker.setPose(Transformation([0.716, -0.4188, 0.0], [0.0149, -0.085, 0.9685, 0.2483]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0859, -0.016, 0.0], [0, 0, 0.998, 0.0176]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=2)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_side(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal_position = Transformation([0, -0.5, 0], [0.00000, 0, 0, 1])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

        final_position = walker.getPose()
        distance_offset = np.linalg.norm((final_position - goal_position.position)[0:2])
        assert distance_offset < 0.15

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=2)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_backward(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal_position = Transformation([-1, 0.3, 0], [0.00000, 0, 0, 1])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        final_position = walker.getPose()
        distance_offset = np.linalg.norm((final_position - goal_position.position)[0:2])
        assert distance_offset < 0.16
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1", "bez2"], indirect=True)
    def test_turn_in_place(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)

        goal = Transformation(euler=[np.pi, 0, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_0(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation(euler=[np.pi / 5, 0, 0])
        goal.position = [0.05, 0.05, 0]
        walker.setGoal(goal)
        # walker.soccerbot.robot_path.show()
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_1(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation(euler=[np.pi, 0, 0])
        goal.position = [0.15, 0.05, 0]
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_2(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation(euler=[np.pi, 0, 0])
        goal.position = [-0.3, 0, 0]
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_3(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation(euler=[-np.pi / 2, 0, 0])
        goal.position = [-0.2, -0.2, 0]
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_4(self, walker: Navigator):
        walker.setPose(Transformation([0.2489, -0.163, 0.0], [0.0284, -0.003, 0.9939, 0.01986]))
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.0503, 0.06323, 0], [0, 0, 1, 0]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_small_movement_5(self, walker: Navigator):
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

    @pytest.mark.timeout(30)
    @pytest.mark.flaky(reruns=1)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_do_nothing(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0.00000, 0, 0, 1]))
        walker.ready()
        walker.wait(100)
        goal = Transformation(euler=[0, 0, 0])
        walker.setGoal(goal)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_tiny_1(self, walker: Navigator):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        walker.setGoal(Transformation([0.01, 0, 0], [0, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_tiny_2(self, walker: Navigator):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        walker.setGoal(Transformation([-0.01, 0, 0], [0, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_tiny_3(self, walker: Navigator):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        walker.setGoal(Transformation([0.01, 0.01, 0], [0, 0, 0, 1]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    @pytest.mark.timeout(30)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_walk_tiny_4(self, walker: Navigator):
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)
        walker.setGoal(Transformation([0, 0, 0], euler=[0.1, 0, 0]))
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

    def test_path_calibration(self):

        start_transform = Transformation([0.0, 0, 0], [0, 0, 0, 1])

        end_transform = Transformation([0.1, 0, 0], [0, 0, 0, 1])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.position[0] > 0.1

        start_transform = Transformation([0.0, 0, 0], [0, 0, 0, 1])

        end_transform = Transformation([0.0, 0, 0], euler=[0.5, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] > 0.5

        end_transform = Transformation([0.0, 0, 0], euler=[-0.5, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] < -0.5

        end_transform = Transformation([0.0, 0, 0], euler=[1.5, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] > 1.5

        end_transform = Transformation([0.0, 0, 0], euler=[-1.5, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] < -1.5

        end_transform = Transformation([0.0, 0, 0], euler=[3.0, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] == np.pi

        end_transform = Transformation([1, 1, 0], euler=[np.pi / 4, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] > np.pi / 4
        assert np.linalg.norm(new_end_transform.position[0:2]) > np.sqrt(2)

        end_transform = Transformation([1, 0, 0], euler=[np.pi / 4, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        assert new_end_transform.orientation_euler[0] > np.pi / 4
        assert np.linalg.norm(new_end_transform.position[0:2]) > 1

    def test_path_calibration_inversion(self):
        rospy.set_param("calibration_trans_a", 1.2)
        rospy.set_param("calibration_trans_b", 0.34)
        rospy.set_param("calibration_trans_a2", 0.67)
        rospy.set_param("calibration_rot_a", 0.75)

        start_transform = Transformation([0.0, 0, 0], [0, 0, 0, 1])
        end_transform = Transformation([1, 1, 0], euler=[np.pi / 4, 0, 0])
        new_end_transform = adjust_navigation_transform(start_transform, end_transform)
        new_new_end_transform = adjust_navigation_transform(start_transform, new_end_transform, invert=True)

        assert abs(new_new_end_transform.position[0] - end_transform.position[0]) < 0.01
        assert abs(new_new_end_transform.position[1] - end_transform.position[1]) < 0.01
        assert abs(new_new_end_transform.orientation_euler[0] - end_transform.orientation_euler[0]) < 0.01

    @pytest.mark.timeout(30)
    @pytest.mark.parametrize("walker", ["bez1"], indirect=True)
    def test_imu_feedback(self, walker: Navigator):
        walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        walker.real_time = False
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([1.5, 0, 0], [0, 0, 0, 1]))

        get_imu_original = walker.soccerbot.get_imu
        pitches = []
        rolls = []
        yaws = []
        locations = [[] for _ in range(8)]
        times = []

        def walker_get_imu_patch():
            imu_transform = get_imu_original()
            pitches.append(imu_transform.orientation_euler[1])
            yaws.append(imu_transform.orientation_euler[0])
            rolls.append(imu_transform.orientation_euler[2])
            times.append(walker.t)
            get_foot_pressure = walker.soccerbot.get_foot_pressure_sensors(walker.ramp.plane)
            for i in range(len(locations)):
                locations[i].append(int(get_foot_pressure[i]))
            return imu_transform

        walker.soccerbot.get_imu = walker_get_imu_patch

        walk_success = walker.run(single_trajectory=True)
        assert walk_success

        def create_angle_plot(angle_name: str, angle_data):
            plt.figure(angle_name)
            plt.plot(times, angle_data, label=f"{angle_name} of robot over time")
            times_after_walk = [t for t in times if t < 0]
            angle_data_after_walk = angle_data[len(times_after_walk) :]

            if angle_name == "Pitches":
                max_angle_offset = round(max(angle_data_after_walk) - walker.soccerbot.walking_pid.setpoint, 5)
                min_angle_offset = round(min(angle_data_after_walk) - walker.soccerbot.walking_pid.setpoint, 5)
                plt.axhline(max(angle_data_after_walk), color="red", label=f"Max {angle_name} Offset {max_angle_offset} rad")
                plt.axhline(min(angle_data_after_walk), color="red", label=f"Min {angle_name} Offset {min_angle_offset} rad")
                plt.axhline(walker.soccerbot.walking_pid.setpoint, color="green", label="Walking set point")
                assert abs(max_angle_offset) < 0.03
                assert abs(min_angle_offset) < 0.03

            times_before_walk = [t for t in times if t < 0]
            angle_data_before_walk = angle_data[0 : len(times_before_walk)]

            if angle_name == "Pitches":
                max_pitch_pre_walk = round(max(angle_data_before_walk), 5)
                min_pitch_pre_walk = round(min(angle_data_before_walk), 5)
                assert abs(max_pitch_pre_walk) < 0.01
                plt.axhline(max(angle_data_before_walk), color="yellow", label=f"Max {angle_name} Pre Walk Offset {max_pitch_pre_walk} rad")
                plt.axhline(min(angle_data_before_walk), color="yellow", label=f"Min {angle_name} Pre Walk Offset {min_pitch_pre_walk} rad")

            plt.xlabel("Time (t)")
            plt.ylabel(f"Forward {angle_name} of robot in radians")
            plt.grid()
            plt.legend()

        def create_foot_pressure_sensor_plot():
            fig, axs = plt.subplots(8)
            fig.suptitle("Foot Pressure Sensor Locations")

            for i in range(len(locations)):
                axs[i].plot(times, locations[i], label="Foot Pressure Sensor %s of robot over time" % (i))

            for ax in axs.flat:
                ax.set(xlabel="Time (t)", ylabel="Sensor Value")
                ax.set_xlim([0, max(times)])
                ax.grid()
                ax.legend()

        create_angle_plot("Pitches", pitches)
        create_angle_plot("Yaws", yaws)
        create_angle_plot("Rolls", rolls)
        create_foot_pressure_sensor_plot()

        if "DISPLAY" in os.environ:
            plt.show()
