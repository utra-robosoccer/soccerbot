import os
import time

os.environ["ROS_NAMESPACE"] = "/robot1"

import numpy as np
import pybullet as pb
import pytest
import rospy

from scipy.interpolate import interp1d

from unittest.mock import MagicMock

from soccer_common.utils_rosparam import set_rosparam_from_yaml_file
from soccer_msgs.msg import FixedTrajectoryCommand
from soccer_trajectories.soccer_trajectories import Trajectory, TrajectoryManager
from soccer_pycontrol.links import Links
from soccer_pycontrol.navigator import Navigator


class TestTrajectory:
    @staticmethod
    @pytest.fixture
    def walker(request) -> Navigator:
        joint_state = MagicMock()
        joint_state.position = [0.0] * 18
        rospy.wait_for_message = MagicMock(return_value=joint_state)

        robot_model = request.param

        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../../soccer_pycontrol/config/" # use config from pycontrol because no config file in trajectories
        config_path = config_folder_path + f"{robot_model}_sim_pybullet.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)
        if "DISPLAY" not in os.environ:
            c = Navigator(display=False, real_time=False)
        else:
            c = Navigator(display=True, real_time=False)

        yield c
        c.close()

    @pytest.mark.parametrize("robot_model", ["bez2"])
    @pytest.mark.parametrize("trajectory_name", ["getupback"])
    def test_all_trajectories(self, robot_model: str, trajectory_name: str):
        rospy.init_node("test")
        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_path = f"{file_path}/../../../{robot_model}_description/config/motor_mapping.yaml"
        set_rosparam_from_yaml_file(param_path=config_path)
        rospy.set_param("robot_model", robot_model)

        if "DISPLAY" not in os.environ:
            Trajectory.RATE = 10000
        c = TrajectoryManager()
        rospy.init_node("test")
        msg = FixedTrajectoryCommand()
        msg.trajectory_name = trajectory_name
        msg.mirror = False
        c.command_callback(command=msg)
        c.trajectory.run(real_time=True)

    # test_trajectory function to test specific trajectories under csv files
    # located in soccer_trajectories/trajectories/{robot_model} in pybullet
    @pytest.mark.parametrize("walker", ["bez2"], indirect=True)
    @pytest.mark.parametrize("robot_model", ["bez2"])  # Just for grabbing the correct path (if can get this info from walker that's better)
    @pytest.mark.parametrize("trajectory_name", ["getupback"])
    def test_trajectory(self, walker: Navigator, robot_model: str, trajectory_name: str):
        # Configure walker and initialize joint array to all 0s
        walker.soccerbot.configuration[
        Links.RIGHT_LEG_1: Links.RIGHT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsRightFoot(
            np.copy(walker.soccerbot.right_foot_init_position)
        )
        walker.soccerbot.configuration[
        Links.LEFT_LEG_1: Links.LEFT_LEG_6 + 1] = walker.soccerbot.inverseKinematicsLeftFoot(
            np.copy(walker.soccerbot.left_foot_init_position)
        )

        pb.setJointMotorControlArray(
            bodyIndex=walker.soccerbot.body,
            controlMode=pb.POSITION_CONTROL,
            jointIndices=list(range(0, 18, 1)),
            targetPositions=[0] * 18,
        )

        FRAMERATE = 100

        # Load CSV data
        curr_path = os.path.dirname(os.path.abspath(__file__))
        trajectories_path = os.path.abspath(os.path.join(curr_path, '..', '..', 'trajectories'))
        csv_file_path = os.path.join(trajectories_path, robot_model, trajectory_name + '.csv')
        with open(csv_file_path, 'r') as file:
            list_data = list(map(lambda s: s.strip().split(','), file.readlines()))

        # Format data and setup interpolation for pybullet
        JOINTS_ARR = [
            "left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
            "left_leg_motor_0", "left_leg_motor_1", "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4",
            "left_leg_motor_5",
            "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3", "right_leg_motor_4",
            "right_leg_motor_5",
            "head_motor_0", "head_motor_1"
        ]
        timestamp_count = len(list_data[0])  # calculate based on number of timestamps in the "time" row (row 0) (excluding time string itself)
        joints_dict = {joint_name: [0] * timestamp_count for joint_name in JOINTS_ARR}

        timestamps = [0] + list(map(lambda s: float(s), list_data[0][1:]))
        framestamps = list(map(lambda f: int(f * 100), timestamps))
        for arr in list_data:
            if arr[0] in joints_dict:
                joints_dict[arr[0]] = [0] + list(map(lambda s: float(s), arr[1:]))
        for key, value in joints_dict.items():
            joints_dict[key] = interp1d(framestamps, value)
        # joints_arr = list(map(lambda n: [joints_dict[key][n] for key in JOINTS_ARR], list(range(timestamp_count))))

        # Run our formatted data in pybullet
        framecount = 0
        total_frames = framestamps[-1]
        additional_frames = FRAMERATE * 3 # 3 extra seconds with no motor movement
        all_frames = list(range(total_frames))
        joints_interp = {key: value(all_frames) for key, value in joints_dict.items()}
        joints_arr = list(map(lambda n: [joints_interp[key][n] for key in JOINTS_ARR], list(range(len(all_frames)))))

        for _ in range(total_frames + additional_frames):
            if framecount < total_frames:
                target_positions = joints_arr[framecount]
                pb.setJointMotorControlArray(
                    bodyIndex=walker.soccerbot.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, 18, 1)),
                    targetPositions=target_positions,
                )
            pb.stepSimulation()
            framecount += 1
            time.sleep(1 / FRAMERATE)