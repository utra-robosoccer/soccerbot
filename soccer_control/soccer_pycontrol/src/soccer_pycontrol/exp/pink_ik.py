#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2023 Inria

"""SigmaBan humanoid standing on two feet."""
from os.path import expanduser

import meshcat_shapes
import numpy as np
import pink
import pinocchio as pin
import qpsolvers
from loop_rate_limiters import RateLimiter
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
from pinocchio import RobotWrapper

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError:
    raise ModuleNotFoundError("Examples need robot_descriptions, " "try `pip install robot_descriptions`")


if __name__ == "__main__":
    # robot = load_robot_description(
    #     "sigmaban_description", root_joint=pin.JointModelFreeFlyer()
    # )
    urdf_model_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_description/bez1_description/urdf/bez1.urdf"
    mesh_dir = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_description/bez1_description"

    robot = RobotWrapper.BuildFromURDF(urdf_model_path, mesh_dir, root_joint=pin.JointModelFreeFlyer())

    viz = pin.visualize.MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
    robot.setVisualizer(viz, init=False)
    viz.initViewer(open=True)
    viz.loadViewerModel()

    configuration = pink.Configuration(robot.model, robot.data, robot.q0)
    viz.display(configuration.q)
    viewer = viz.viewer

    left_foot_task = FrameTask(
        "/left_foot",
        position_cost=1.0,
        orientation_cost=1.0,
    )
    torso_task = FrameTask(
        "/torso",
        position_cost=1.0,
        orientation_cost=1.0,
    )
    right_foot_task = FrameTask(
        "/right_foot",
        position_cost=1.0,
        orientation_cost=1.0,
    )
    posture_task = PostureTask(
        cost=1e-2,  # [cost] / [rad]
    )
    tasks = [left_foot_task, torso_task, right_foot_task, posture_task]

    torso_pose = configuration.get_transform_frame_to_world("/torso").copy()
    # torso_pose.translation[0] += 0.05
    torso_task.set_target(torso_pose)
    posture_task.set_target_from_configuration(configuration)

    transform_left_foot_tip_target_to_init = pin.SE3(np.eye(3), np.array([0.1, 0.13, 0.0]))
    transform_right_foot_tip_target_to_init = pin.SE3(np.eye(3), np.array([-0.1, -0.13, 0.0]))

    left_foot_task.set_target(configuration.get_transform_frame_to_world("/left_foot") * transform_left_foot_tip_target_to_init)
    right_foot_task.set_target(configuration.get_transform_frame_to_world("/right_foot") * transform_right_foot_tip_target_to_init)
    torso_task.set_target(configuration.get_transform_frame_to_world("/torso"))

    # Display targets
    meshcat_shapes.frame(viewer["left_foot_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["right_foot_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["torso_target"], opacity=0.5)
    viewer["left_foot_target"].set_transform(left_foot_task.transform_target_to_world.np)
    viewer["right_foot_target"].set_transform(right_foot_task.transform_target_to_world.np)
    viewer["torso_target"].set_transform(torso_task.transform_target_to_world.np)

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "quadprog" in qpsolvers.available_solvers:
        solver = "quadprog"

    rate = RateLimiter(frequency=200.0)
    dt = rate.period
    t = 0.0  # [s]
    while True:
        # Compute velocity and integrate it into next configuration
        import time

        s = time.time()
        velocity = solve_ik(configuration, tasks, dt, solver=solver)
        configuration.integrate_inplace(velocity, dt)
        print((time.time() - s) * 1000)

        # Visualize result at fixed FPS
        viz.display(configuration.q)
        rate.sleep()
        t += dt
