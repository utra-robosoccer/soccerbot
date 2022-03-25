#!/usr/bin/env python3

import glob
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import numpy as np

from soccer_common.transformation import Transformation

import rospy
import soccerbot_controller_ros


if __name__ == '__main__':

    rospy.init_node("soccer_control")
    for node in ['soccer_strategy', 'soccer_pycontrol', 'soccer_trajectories', 'ball_detector', 'detector_goalpost', 'object_detector', 'amcl']:
        os.system(f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/{node}'")

    np.set_printoptions(precision=3)
    clear = True
    x_range = np.flip(np.array([-1.0, -0.5, -0.3, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.3, 0.4, 0.5, 1.0]))
    y_range = np.array([0, 0.05, 0.1, 0.15, 0.3, 0.4, 0.5, 1.0, 1.5])
    # ang_range = np.pi * np.array([-0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8])
    ang_range = np.pi * np.array([0])

    if clear:
        files = glob.glob('calibration/*')
        for f in files:
            os.remove(f)

    for x in x_range:
        for y in y_range:
            for yaw in ang_range:
                file_name = f"calibration/{x:.2f}_{y:.2f}_{yaw:.2f}.npy"
                if os.path.exists(file_name):
                    continue

                quat = Transformation.get_quaternion_from_euler([yaw, 0, 0])

                print(f"Getting Calibration for x: {x} y: {y} yaw: {yaw:.2f}")

                attempt = 0
                while attempt < 5:
                    walker = soccerbot_controller_ros.SoccerbotControllerRos()
                    walker.setPose(Transformation())
                    walker.ready()
                    walker.wait(300)
                    walker.setGoal(Transformation([x, y, 0.0], quat))
                    success = walker.run(single_trajectory=True)
                    if success:
                        break
                    else:
                        print("Attempt Failed")
                    attempt = attempt + 1

                if attempt == 5:
                    print("Failed 5 times")