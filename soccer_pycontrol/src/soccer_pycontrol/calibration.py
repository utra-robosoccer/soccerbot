#!/usr/bin/env python3
import copy
import os
import sys

import ruamel.yaml
from scipy.optimize import curve_fit
from scipy.stats import stats

from soccer_pycontrol.utils import trimToPi, wrapToPi

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import numpy as np
import rospy

from soccer_common.transformation import Transformation

robot_model = "bez1"


def setup_calibration():
    from os.path import exists
    from unittest.mock import MagicMock

    import yaml

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

    pass


def calibrate_x():
    setup_calibration()

    import pybullet as pb

    from soccer_pycontrol.soccerbot_controller import SoccerbotController

    start_positions = []
    final_positions = []
    for x in np.linspace(0.0, 0.2, 21):
        walker = SoccerbotController(display=False, useCalibration=False)

        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)

        actual_start_position = np.array(
            [
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][0],
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][1],
                Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(walker.soccerbot.body)[1])[0],
            ]
        )

        goal_position = Transformation([x, 0, 0])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

        final_position = np.array(
            [
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][0],
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][1],
                Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(walker.soccerbot.body)[1])[0],
            ]
        )
        final_transformation = final_position - actual_start_position

        print(f"Final Transformation {final_transformation}")

        start_positions.append(goal_position.get_position())
        final_positions.append(final_transformation)

        walker.wait(100)

        del walker

    # Plot the beginning and final positions
    import matplotlib.pyplot as plt

    x = np.array(start_positions)[:, 0]
    y = np.array(final_positions)[:, 0]
    plt.scatter(x, y, marker="+")
    plt.title("X position vs Actual X position")
    plt.xlabel("Desired X Position")
    plt.ylabel("Actual X Position")
    plt.grid(b=True, which="both", color="0.65", linestyle="-")
    plt.minorticks_on()

    def func(x, a, b):
        return a * (x**2) + b * x

    popt, pcov = curve_fit(func, x, y)
    plt.plot(x, func(x, *popt), "r-", label="fit: a=%5.3f, b=%5.3f" % tuple(popt))
    plt.show()

    def func(x, a):
        return a * x

    popt2, pcov2 = curve_fit(func, x[int(len(x) / 2) : -1], y[int(len(y) / 2) : -1])

    return popt[0], popt[1], popt2[0]


def calibrate_theta():
    setup_calibration()

    import pybullet as pb

    from soccer_pycontrol.soccerbot_controller import SoccerbotController

    start_angles = []
    final_angles = []
    for theta in np.linspace(-np.pi / 2, np.pi / 2, 21):
        walker = SoccerbotController(display=False, useCalibration=False)

        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)

        actual_start_position = np.array(
            [
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][0],
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][1],
                Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(walker.soccerbot.body)[1])[0],
            ]
        )

        goal_position = Transformation([0, 0, 0], Transformation.get_quaternion_from_euler([theta, 0, 0]))
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        assert walk_success

        final_position = np.array(
            [
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][0],
                pb.getBasePositionAndOrientation(walker.soccerbot.body)[0][1],
                Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(walker.soccerbot.body)[1])[0],
            ]
        )
        final_transformation = final_position - actual_start_position

        print(f"Final Transformation {final_transformation}")

        start_angles.append(theta)
        final_angles.append(final_transformation[2])

        walker.wait(100)

        del walker

    # Plot the beginning and final positions
    import matplotlib.pyplot as plt

    x = np.array(start_angles)
    y = np.array(final_angles)
    plt.scatter(x, y, marker="+")
    plt.title("Theta vs Actual Theta position")
    plt.xlabel("Desired Theta Position")
    plt.ylabel("Actual Theta Position")
    plt.grid(b=True, which="both", color="0.65", linestyle="-")
    plt.minorticks_on()

    def func(x, a):
        return a * x

    popt, pcov = curve_fit(func, x, y)
    plt.plot(x, func(x, *popt), "r-", label="fit: a=%5.3f" % tuple(popt))
    plt.show()

    return popt[0]


def adjust_navigation_transform(start_transform: Transformation, end_transform: Transformation) -> Transformation:
    calibration_trans_a = rospy.get_param("calibration_trans_a", 0)
    calibration_trans_b = rospy.get_param("calibration_trans_b", 1)
    calibration_trans_a2 = rospy.get_param("calibration_trans_a2", 1)
    calibration_rot_a = rospy.get_param("calibration_rot_a", 1)

    def isWalkingBackwards():
        start_angle = start_transform.get_orientation_euler()[0]
        del_pose = end_transform.get_position() - start_transform.get_position()
        if np.dot([np.cos(start_angle), np.sin(start_angle)], del_pose[0:2]) < 0:
            return True
        return False

    diff_position = end_transform.get_position()[0:2] - start_transform.get_position()[0:2]
    start_angle = start_transform.get_orientation_euler()[0]
    intermediate_angle = np.arctan2(diff_position[1], diff_position[0])

    if isWalkingBackwards():
        intermediate_angle = wrapToPi(intermediate_angle + np.pi)
    final_angle = end_transform.get_orientation_euler()[0]

    step_1_angular_distance = wrapToPi(intermediate_angle - start_angle)
    step_2_distance = np.linalg.norm(diff_position)
    step_3_angular_distance = wrapToPi(final_angle - intermediate_angle)

    step_1_angular_distance_new = (1 / calibration_rot_a) * step_1_angular_distance
    step_3_angular_distance_new = (1 / calibration_rot_a) * step_3_angular_distance

    max_x = calibration_trans_a * 0.2**2 + calibration_trans_b * 0.2
    c = min(max_x, step_2_distance)
    c_remainder = max(0.0, step_2_distance - max_x)

    quadratic = lambda a, b, c: (-b + np.sqrt(b**2 + 4 * a * c)) / (2 * a)
    step_2_distance_new = quadratic(calibration_trans_a, calibration_trans_b, c)
    step_2_distance_new = step_2_distance_new + (1 / calibration_trans_a2) * c_remainder

    step_1_angular_distance_new = trimToPi(step_1_angular_distance_new)
    step_3_angular_distance_new = trimToPi(step_3_angular_distance_new)

    rot1 = Transformation((0, 0, 0), Transformation.get_quaternion_from_euler([step_1_angular_distance_new, 0, 0]))
    trans = Transformation((step_2_distance_new, 0, 0))
    rot2 = Transformation((0, 0, 0), Transformation.get_quaternion_from_euler([step_3_angular_distance_new, 0, 0]))

    end_transform_new = start_transform @ rot1 @ trans @ rot2

    return end_transform_new


if __name__ == "__main__":
    config_file_path = os.path.dirname(__file__).replace("src/soccer_pycontrol", f"config/{robot_model}_sim.yaml")
    yaml = ruamel.yaml.YAML()

    # Calibrate translation
    a, b, a2 = calibrate_x()
    with open(config_file_path) as f:
        data = yaml.load(f)
    data["calibration_trans_a"] = float(a)
    data["calibration_trans_b"] = float(b)
    data["calibration_trans_a2"] = float(a2)
    with open(config_file_path, "w") as f:
        yaml.dump(data, f)

    # Calibrate rotation
    a = calibrate_theta()
    with open(config_file_path) as f:
        data = yaml.load(f)
    data["calibration_rot_a"] = float(a)
    with open(config_file_path, "w") as f:
        yaml.dump(data, f)
