#!/usr/bin/env python3
import copy
import functools
import glob
import os

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import argparse

import matplotlib.pyplot as plt
import numpy as np
import pybullet as pb
import rospy
import torch
from torch import nn

from soccer_common.transformation import Transformation


class Calibration:
    x_range = np.linspace(0, 0.5, 25)
    y_range = np.linspace(-0.5, 0.5, 50)
    ang_range = np.pi * np.linspace(-0.9, 0.9, 20)
    try:
        robot_model = rospy.get_param("~robot_model", "bez1")
    except ConnectionRefusedError:
        robot_model = "bez1"

    # Runs a series of movements to collect data in the calibration folder
    def obtain_calibration(self):
        from soccer_pycontrol.soccerbot_controller import SoccerbotController

        try:
            np.set_printoptions(precision=3)

            if not os.path.exists("calibration"):
                os.makedirs("calibration")

            clear = False
            if clear:
                files = glob.glob("calibration/*")
                for f in files:
                    os.remove(f)

            for x in Calibration.x_range:
                for y in Calibration.y_range:
                    for yaw in Calibration.ang_range:
                        file_name = f"calibration/{x:.2f}_{y:.2f}_{yaw:.2f}.npy"
                        if os.path.exists(file_name):
                            continue

                        quat = Transformation.get_quaternion_from_euler([yaw, 0, 0])

                        print(f"Getting Calibration for x: {x} y: {y} yaw: {yaw:.2f}")

                        attempt = 0
                        while attempt < 5:
                            walker = SoccerbotController(display=False)
                            walker.soccerbot.useCalibration = False

                            walker.setPose(Transformation([0.0, 0, 0], [0, 0, 0, 1]))
                            walker.ready()
                            walker.wait(100)
                            walker.setGoal(Transformation([x, y, 0.0], quat))
                            success = walker.run(single_trajectory=True)
                            if success:
                                final_position = [
                                    pb.getBasePositionAndOrientation(walker.soccerbot.body)[0],
                                    pb.getBasePositionAndOrientation(walker.soccerbot.body)[1],
                                    Transformation.get_euler_from_quaternion(pb.getBasePositionAndOrientation(walker.soccerbot.body)[1])[0],
                                ]
                                print("Final Position is " + str(final_position))
                                np.save(file_name, final_position)
                                break
                            else:
                                print("Attempt Failed")
                            attempt = attempt + 1

                        del walker
                        if attempt == 5:
                            print("Failed 5 times")
                            exit(1)

            exit(0)
        except KeyboardInterrupt:
            exit(1)
        except rospy.exceptions.ROSException:
            exit(1)

    def load_data(self, combine_yaw=False):
        files = glob.glob("calibration/*")
        goal_positions = []
        end_positions = []
        for f in files:
            goal_position = np.array(f.replace("calibration/", "").replace(".npy", "").split("_")).astype(float)
            end_position = np.load(f)

            if combine_yaw:
                has = False
                for i in range(len(goal_positions)):
                    if goal_positions[i][0] == goal_position[0] and goal_positions[i][1] == goal_position[1]:
                        end_positions[i][0:2] = end_positions[i][0:2] + end_position[0:2]
                        has = True
                if not has:
                    goal_positions.append(goal_position)
                    end_positions.append(end_position)

            else:
                goal_positions.append(goal_position)
                end_positions.append(end_position)

        goal_positions = np.array(goal_positions)
        end_positions = np.array(end_positions)

        if combine_yaw:
            end_positions[:, 0:2] = end_positions[:, 0:2] / len(Calibration.ang_range)
            goal_positions[:, 2] = 0
            end_positions[:, 2] = 0

        return goal_positions, end_positions

    def calibrate(self):
        last_epoch = 0
        model_name = f"calibration_models/model.{last_epoch}"

        goal_positions, end_positions = self.load_data(combine_yaw=True)

        # define the model
        class NN(nn.Module):
            def __init__(self):
                super(NN, self).__init__()
                self.flatten = nn.Flatten()
                self.linear_relu_stack = nn.Sequential(
                    nn.Linear(2, 64),
                    nn.ReLU(),
                    nn.Linear(64, 64),
                    nn.ReLU(),
                    nn.Linear(64, 2),
                )

            def forward(self, inputs):
                outputs = self.linear_relu_stack(inputs)
                return outputs

        model = NN()
        if os.path.exists(model_name):
            print("Loading from model " + model_name)
            model.load_state_dict(torch.load(model_name))

        # optimizer and per-prediction error
        criterion = torch.nn.MSELoss(reduction="sum")
        optimizer = torch.optim.SGD(model.parameters(), lr=0.003)

        # for stochastic gradient descent, create batches
        train_dataset = torch.utils.data.DataLoader((goal_positions[:, 0:2], end_positions[:, 0:2]), batch_size=2, shuffle=True, num_workers=4)

        epochs = 300
        for epoch in range(last_epoch, epochs):
            for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):

                predictions = model(x_batch_train.float())
                loss = criterion(y_batch_train.float(), predictions)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            print(f"Epoch {epoch}: Average ppError: {loss.item()}")
            if epoch % 5 == 0:
                torch.save(model.state_dict(), f"calibration_models/model.{epoch}")

    def view_calibration(self):
        goal_positions, end_positions = self.load_data(True)

        # plt.quiver(end_positions[:,0], end_positions[:,1], np.cos(end_positions[:,2]), np.sin(end_positions[:,2]), color="red", width=0.001)
        plt.quiver(
            end_positions[:, 0],
            end_positions[:, 1],
            goal_positions[:, 0] - end_positions[:, 0],
            goal_positions[:, 1] - end_positions[:, 1],
            color="green",
            scale=1,
            scale_units="xy",
            angles="xy",
            width=0.0003,
        )
        plt.quiver(
            goal_positions[:, 0],
            goal_positions[:, 1],
            np.cos(goal_positions[:, 2]),
            np.sin(goal_positions[:, 2]),
            width=0.001,
        )

        if os.path.exists(self.model_name):
            print("Loading from model " + self.model_name)
            model = torch.load(self.model_name)
            end_positions_model = np.array(model(goal_positions[:, 0:2]))
            # plt.quiver(end_positions_model[:, 0], end_positions_model[:, 1], np.cos(end_positions_model[:, 2]),
            #            np.sin(end_positions_model[:, 2]), color="orange", width=0.001)
            plt.quiver(
                goal_positions[:, 0],
                goal_positions[:, 1],
                end_positions_model[:, 0] - goal_positions[:, 0],
                end_positions_model[:, 1] - goal_positions[:, 1],
                color="red",
                scale=1,
                scale_units="xy",
                angles="xy",
                width=0.0003,
            )
            # plt.quiver(end_positions_model[:, 0], end_positions_model[:, 1],
            #            end_positions[:, 0] - end_positions_model[:, 0],
            #            end_positions[:, 1] - end_positions_model[:, 1],
            #            color="purple", scale=1, scale_units='xy', angles='xy', width=0.0003)

        plt.show()
        pass

    def invert_model(self):
        precision = 0.1
        print("Loading from model " + self.model_name)
        model = torch.load(self.model_name)

        points = []
        for x in np.arange(-2, 2, precision):
            for y in np.arange(0, 1, precision):
                points.append([x, y])

        end_points = np.array(points)
        start_points = np.array(model(end_points))
        end_points_filter = []
        start_points_filter = []
        for start_point, end_point in zip(start_points, end_points):
            if start_point[1] < 0:
                continue

            start_points_filter.append(start_point)
            end_points_filter.append(end_point)

            start_point_neg = np.copy(start_point)
            end_point_neg = np.copy(end_point)
            start_point_neg[1] = -start_point_neg[1]
            end_point_neg[1] = -end_point_neg[1]
            start_points_filter.append(start_point_neg)
            end_points_filter.append(end_point_neg)

        end_points_filter = np.array(end_points_filter)
        start_points_filter = np.array(start_points_filter)

        plt.quiver(
            start_points_filter[:, 0],
            start_points_filter[:, 1],
            end_points_filter[:, 0] - start_points_filter[:, 0],
            end_points_filter[:, 1] - start_points_filter[:, 1],
            width=0.0002,
            scale=1,
            scale_units="xy",
            angles="xy",
        )
        plt.show()

        class NN(nn.Module):
            def __init__(self):
                super(NN, self).__init__()
                self.flatten = nn.Flatten()
                self.linear_relu_stack = nn.Sequential(
                    nn.Linear(2, 64),
                    nn.ReLU(),
                    nn.Linear(64, 64),
                    nn.ReLU(),
                    nn.Linear(64, 2),
                )

            def forward(self, inputs):
                outputs = self.linear_relu_stack(inputs)
                return outputs

        model = NN()

        # optimizer and per-prediction error
        criterion = torch.nn.MSELoss(reduction="sum")
        optimizer = torch.optim.SGD(model.parameters(), lr=0.003)

        # for stochastic gradient descent, create batches
        batch_size = 1
        train_dataset = torch.utils.data.DataLoader((start_points_filter, end_points_filter), batch_size=2, shuffle=True, num_workers=4)

        epochs = 21
        for epoch in range(0, epochs):
            for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):

                predictions = model(x_batch_train.float())
                loss = criterion(y_batch_train.float(), predictions)

                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

            print(f"Epoch {epoch}: Average ppError: {loss.item()}")
        torch.save(model, self.model_name + "_inverse")

    @functools.cached_property
    def adjust_navigation_goal(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        calib_file = dir_path + "/" + self.model_name + "_inverse"
        try:
            calib_file = rospy.get_param("~calibration_file", calib_file)
        except ConnectionRefusedError as ce:
            pass

        print("Loading from model " + calib_file)
        model = torch.load(calib_file)
        return model

    def test_adjust_navigation_goal(self):
        points = []
        precision = 0.1
        for x in np.arange(-1, 1, precision):
            for y in np.arange(-1, 1, precision):
                points.append([x, y])

        start_points = np.array(points)
        end_points = np.array(self.adjust_navigation_goal(start_points))

        plt.quiver(
            start_points[:, 0],
            start_points[:, 1],
            end_points[:, 0] - start_points[:, 0],
            end_points[:, 1] - start_points[:, 1],
            width=0.0005,
            scale=1,
            scale_units="xy",
            angles="xy",
        )
        plt.show()

    def adjust_navigation_transform(self, start_transform: Transformation, end_transform: Transformation) -> Transformation:
        if np.linalg.norm(start_transform.get_transform()[0:2] - end_transform.get_transform()[0:2]) < 0.05:
            return end_transform

        diff_transform = np.linalg.inv(start_transform) @ end_transform
        diff_position = diff_transform[0:2, 3]
        [diff_yaw, diff_pitch, diff_roll] = Transformation.get_euler_from_rotation_matrix(diff_transform[0:3, 0:3])
        scale = 1
        boundary = 0.7
        if abs(diff_position[0]) > boundary:
            scale = abs(diff_position[0]) / boundary
        elif abs(diff_position[1]) > boundary:
            scale = abs(diff_position[1]) / boundary

        diff_position = diff_position / scale

        yaw_scale = 1
        new_position = np.array(self.adjust_navigation_goal(np.array([diff_position])))[0]
        new_position = new_position * scale

        diff_yaw = max(min(diff_yaw * yaw_scale, np.pi), -np.pi)

        diff_transform_new = copy.deepcopy(diff_transform)
        diff_transform_new[0:2, 3] = new_position
        diff_transform_new.set_orientation(Transformation.get_quaternion_from_euler([diff_yaw, 0, 0]))
        end_transform_new = start_transform @ diff_transform_new
        return end_transform_new

    def test_adjust_navigation_transform(self):
        transforms = []
        precision = 0.1
        for x in np.arange(-2, 2, precision):
            for y in np.arange(-2, 2, precision):
                t = Transformation(position=[x, y, 0])
                transforms.append(t)

        transforms_adjusted = []
        start = Transformation()
        for transform in transforms:
            transforms_adjusted.append(self.adjust_navigation_transform(start, transform))

        start_points = []
        end_points = []
        for transform in transforms:
            start_points.append(transform.get_position()[0:2])
        for transform in transforms_adjusted:
            end_points.append(transform.get_position()[0:2])

        start_points = np.array(start_points)
        end_points = np.array(end_points)

        plt.quiver(
            start_points[:, 0],
            start_points[:, 1],
            end_points[:, 0] - start_points[:, 0],
            end_points[:, 1] - start_points[:, 1],
            width=0.0005,
            scale=1,
            scale_units="xy",
            angles="xy",
        )
        plt.show()
