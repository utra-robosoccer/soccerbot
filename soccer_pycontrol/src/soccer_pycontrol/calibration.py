#!/usr/bin/env python3
import copy
import os
import sys

import ruamel.yaml
from scipy.optimize import curve_fit
from scipy.stats import stats

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
        return a * x + b * x**2

    popt, pcov = curve_fit(func, x, y)
    plt.plot(x, func(x, *popt), "r-", label="fit: a=%5.3f, b=%5.3f" % tuple(popt))
    plt.show()

    return popt[0], popt[1]


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

    return popt[0], popt[1]


def adjust_navigation_transform(start_transform: Transformation, end_transform: Transformation) -> Transformation:
    x_navigation_scale = rospy.get_param("x_navigation_scale", 1.3)
    y_navigation_scale = rospy.get_param("y_navigation_scale", 1)
    theta_navigation_scale = rospy.get_param("theta_navigation_scale", 1)

    if np.linalg.norm(start_transform.get_transform()[0:2] - end_transform.get_transform()[0:2]) < 0.05:
        return end_transform

    diff_transform = np.linalg.inv(start_transform) @ end_transform
    diff_position = diff_transform[0:2, 3]
    [diff_yaw, diff_pitch, diff_roll] = Transformation.get_euler_from_rotation_matrix(diff_transform[0:3, 0:3])

    new_position = diff_position
    new_position[0] *= x_navigation_scale
    new_position[1] *= y_navigation_scale

    diff_yaw = max(min(diff_yaw * theta_navigation_scale, np.pi), -np.pi)

    diff_transform_new = copy.deepcopy(diff_transform)
    diff_transform_new[0:2, 3] = new_position
    diff_transform_new.set_orientation(Transformation.get_quaternion_from_euler([diff_yaw, 0, 0]))
    end_transform_new = start_transform @ diff_transform_new
    return end_transform_new


if __name__ == "__main__":
    config_file_path = os.path.dirname(__file__).replace("src/soccer_pycontrol", f"config/{robot_model}_sim.yaml")
    yaml = ruamel.yaml.YAML()

    # Calibrate translation
    a, b = calibrate_x()
    with open(config_file_path, "r+") as f:
        data = yaml.load(f)
        data["calibration_trans_a"] = a
        data["calibration_trans_b"] = b
    with open(config_file_path, "wb") as f:
        yaml.dump(data, f)

    # Calibrate rotation
    a = calibrate_theta()
    with open(config_file_path, "r+") as f:
        yaml = ruamel.yaml.YAML()
        data = yaml.load(f)
        data["calibration_rot_a"] = a
        yaml.dump(data, f)
