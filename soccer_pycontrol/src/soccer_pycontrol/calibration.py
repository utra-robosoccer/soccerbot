#!/usr/bin/env python3
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import sys

import numpy as np
import rospy
import ruamel.yaml
from scipy.optimize import curve_fit

from soccer_common.transformation import Transformation
from soccer_pycontrol.utils import trimToPi, wrapToPi

robot_model = "bez1"
run_in_ros = False


def setup_calibration():
    from os.path import exists
    from unittest.mock import MagicMock

    import yaml

    sys.modules["rospy"] = MagicMock()
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
    if not run_in_ros:
        setup_calibration()

    from soccer_pycontrol.navigator import Navigator
    from soccer_pycontrol.navigator_ros import NavigatorRos

    start_positions = []
    final_positions = []
    for x in np.linspace(0.0, 0.25, 26):
        if run_in_ros:
            walker = NavigatorRos(useCalibration=False)
        else:
            walker = Navigator(display=False, real_time=False, useCalibration=False)
        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()

        actual_start_position = walker.getPose()

        goal_position = Transformation([x, 0, 0])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        if not walk_success:
            continue

        final_position = walker.getPose()
        final_transformation = final_position - actual_start_position

        print(f"Final Transformation {final_transformation}")

        start_positions.append(goal_position.position)
        final_positions.append(final_transformation)

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

    fs = 8  # Final samples used to approximate linear fit after 0.25
    popt2, pcov2 = curve_fit(func, x[-fs:] - x[-fs], y[-fs:] - y[-fs])

    return popt[0], popt[1], popt2[0]


def calibrate_theta():
    if not run_in_ros:
        setup_calibration()

    from soccer_pycontrol.navigator import Navigator
    from soccer_pycontrol.navigator_ros import NavigatorRos

    start_angles = []
    final_angles = []
    for theta in np.linspace(-np.pi / 2, np.pi / 2, 21):
        if run_in_ros:
            walker = NavigatorRos(useCalibration=False)
        else:
            walker = Navigator(display=False, useCalibration=False)

        walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
        walker.ready()
        walker.wait(200)

        actual_start_position = walker.getPose()

        goal_position = Transformation([0, 0, 0], euler=[theta, 0, 0])
        walker.setGoal(goal_position)
        walk_success = walker.run(single_trajectory=True)
        if not walk_success:
            continue

        final_position = walker.getPose()
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
        start_angle = start_transform.orientation_euler[0]
        del_pose = end_transform.position - start_transform.position
        if np.dot([np.cos(start_angle), np.sin(start_angle)], del_pose[0:2]) < 0:
            return True
        return False

    diff_position = end_transform.position[0:2] - start_transform.position[0:2]
    start_angle = start_transform.orientation_euler[0]
    intermediate_angle = np.arctan2(diff_position[1], diff_position[0])

    if isWalkingBackwards():
        intermediate_angle = wrapToPi(intermediate_angle + np.pi)
    final_angle = end_transform.orientation_euler[0]

    step_1_angular_distance = wrapToPi(intermediate_angle - start_angle)
    step_2_distance = np.linalg.norm(diff_position)
    step_3_angular_distance = wrapToPi(final_angle - intermediate_angle)

    step_1_angular_distance_new = (1 / calibration_rot_a) * step_1_angular_distance
    step_3_angular_distance_new = (1 / calibration_rot_a) * step_3_angular_distance

    max_x = calibration_trans_a * 0.25**2 + calibration_trans_b * 0.25
    c = min(max_x, step_2_distance)
    c_remainder = max(0.0, step_2_distance - max_x)

    quadratic = lambda a, b, c: (-b + np.sqrt(b**2 + 4 * a * c)) / (2 * a) if a != 0 else (1 / b) * c
    step_2_distance_new = quadratic(calibration_trans_a, calibration_trans_b, c)
    step_2_distance_new = step_2_distance_new + (1 / calibration_trans_a2) * c_remainder

    step_1_angular_distance_new = trimToPi(step_1_angular_distance_new)
    step_3_angular_distance_new = trimToPi(step_3_angular_distance_new)

    rot1 = Transformation((0, 0, 0), euler=[step_1_angular_distance_new, 0, 0])
    trans = Transformation((step_2_distance_new, 0, 0)) if not isWalkingBackwards() else Transformation((-step_2_distance_new, 0, 0))
    rot2 = Transformation((0, 0, 0), euler=[step_3_angular_distance_new, 0, 0])

    end_transform_new = start_transform @ rot1 @ trans @ rot2
    end_transform_new.is_walking_backwards = isWalkingBackwards()  # HACK needed to pass the calibrated walking over :(

    return end_transform_new


if __name__ == "__main__":
    if run_in_ros:
        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )
        rospy.init_node("soccer_control_calibration")
        rospy.loginfo("Initializing Soccer Control Calibration")
        config_file_path = os.path.dirname(__file__).replace("src/soccer_pycontrol", f"config/{robot_model}_sim.yaml")
    else:
        config_file_path = os.path.dirname(__file__).replace("src/soccer_pycontrol", f"config/{robot_model}_sim_pybullet.yaml")

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
