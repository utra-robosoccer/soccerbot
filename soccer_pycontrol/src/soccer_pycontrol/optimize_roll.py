import os

import rospy

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"
import numpy as np
import pybullet as pb
import pytest
from matplotlib import pyplot as plt

from soccer_common.mock_ros import mock_ros
from soccer_common.transformation import Transformation
from soccer_pycontrol.calibration import adjust_navigation_transform
from soccer_pycontrol.links import Links
from soccer_pycontrol.navigator import Navigator
from soccer_pycontrol.navigator_ros import NavigatorRos

rospy.init_node("test")

kp_max = 0.5
kd_max = 1
# kp_list = np.arange(0, kp_max, kp_max / 20).tolist()
# kd_list = np.arange(0, kd_max, kd_max / 20).tolist()
kp_list = np.arange(0, kp_max, kp_max / 4).tolist()
kd_list = np.arange(0, kd_max, kd_max / 4).tolist()

walker = Navigator()
get_imu_original = walker.soccerbot.get_imu


def walker_get_imu_patch():
    imu_transform = get_imu_original()
    rolls.append(imu_transform.orientation_euler[2])
    times.append(walker.t)
    return imu_transform


walker.soccerbot.get_imu = walker_get_imu_patch

length_kp = len(kp_list)
length_kd = len(kd_list)
max_of_all_list = np.zeros((length_kp, length_kd))
for i in range(length_kp):
    for j in range(length_kd):
        kp = kp_list[i]
        kd = kd_list[j]
        walker.setPose(Transformation([0, 0, 0], [0, 0, 0, 1]))
        walker.real_time = False
        walker.ready()
        walker.wait(100)
        walker.setGoal(Transformation([0.5, 0, 0], [0, 0, 0, 1]))
        rolls = []
        times = []

        walk_success = walker.run(single_trajectory=True)
        if not walk_success:
            continue
        times_after_walk = [t for t in times if t < 0]
        rolls_after_walk = rolls[len(times_after_walk) :]
        max_roll_offset = round(max(rolls_after_walk) - walker.soccerbot.walking_pid.setpoint, 5)
        min_roll_offset = round(min(rolls_after_walk) - walker.soccerbot.walking_pid.setpoint, 5)
        # assert abs(max_roll_offset) < 0.2
        # assert abs(min_roll_offset) < 0.2
        max_of_all = max(abs(max_roll_offset), abs(min_roll_offset))
        max_of_all_list[i][j] = max_of_all
        print(max_of_all)
fig = plt.figure()
ax = plt.axes(projection="3d")
ax.plot_wireframe(kp_list, kd_list, max_of_all_list, color="black")
ax.set_title("wireframe")
plt.show()
