#!/usr/bin/env python3
import copy
import functools
import glob
import os

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "1"

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import argparse

import keras
import matplotlib.pyplot as plt
import numpy as np
import rospy
import tensorflow as tf
from keras.utils.vis_utils import plot_model

from soccer_common.transformation import Transformation


class Calibration:
    # fmt: off
    x_range = np.flip(
        np.array([-2.0, -1.5, -1.0, -0.8, -0.5, -0.3, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.3, 0.4, 0.5, 0.8, 1.0, 1.5, 2.0]))
    y_range = np.array([0, 0.05, 0.1, 0.15, 0.3, 0.4, 0.5, 1.0, 1.5, 2.0])
    ang_range = np.pi * np.array([-0.8, -0.6, -0.4, -0.2, 0, 0.2, 0.4, 0.6, 0.8, 1])
    # fmt: on
    model_name = "calibration_models/model.995"

    # Runs a series of movements to collect data in the calibration folder
    def obtain_calibration(self):
        from soccer_pycontrol.soccerbot_controller_ros import SoccerbotControllerRos

        rospy.init_node("soccer_control")
        for node in [
            "soccer_strategy",
            "soccer_pycontrol",
            "soccer_trajectories",
            "ball_detector",
            "detector_goalpost",
            "object_detector",
            "rosbag",
        ]:
            os.system(f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/{node}'")

        try:
            np.set_printoptions(precision=3)

            # ang_range = np.pi * np.array([0])
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
                            walker = SoccerbotControllerRos()
                            walker.soccerbot.useCalibration = False
                            walker.setPose(Transformation())
                            walker.setGoal(Transformation([x, y, 0.0], quat))
                            success = walker.run(single_trajectory=True)
                            if success:
                                walker.update_robot_pose(footprint_name="/base_footprint_gt")
                                euler = Transformation.get_euler_from_quaternion(
                                    [
                                        walker.robot_pose.pose.orientation.x,
                                        walker.robot_pose.pose.orientation.y,
                                        walker.robot_pose.pose.orientation.z,
                                        walker.robot_pose.pose.orientation.w,
                                    ]
                                )
                                final_position = [
                                    walker.robot_pose.pose.position.x,
                                    walker.robot_pose.pose.position.y,
                                    euler[0],
                                ]
                                print("Final Position is " + str(final_position))
                                np.save(file_name, final_position)
                                break
                            else:
                                print("Attempt Failed")
                            attempt = attempt + 1

                        if attempt == 5:
                            print("Failed 5 times")
                            exit(1)

            exit(0)
        except (KeyboardInterrupt, rospy.exceptions.ROSException):
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
        if os.path.exists(model_name):
            print("Loading from model " + model_name)
            model = keras.models.load_model(model_name)
        else:
            inputs = keras.Input(shape=(2), name="end_positions")
            x1 = tf.keras.layers.Dense(64, activation="leaky_relu")(inputs)
            x2 = tf.keras.layers.Dense(64, activation="leaky_relu")(x1)
            outputs = tf.keras.layers.Dense(2, name="goal_positions")(x2)
            model = keras.Model(inputs=inputs, outputs=outputs)
            model.compile(loss="mse", optimizer=tf.keras.optimizers.Adam(learning_rate=0.003))

        # plot it
        plot_model(
            model,
            to_file="model.png",
            show_shapes=True,
            show_dtype=True,
            show_layer_names=True,
            rankdir="TB",  # TB: vertical; LR: hor
            expand_nested=True,
            dpi=96,
        )

        # optimizer and per-prediction error
        optimizer = tf.keras.optimizers.SGD(learning_rate=0.003)
        f_ppError = tf.keras.losses.MeanSquaredError()

        # for stochastic gradient descent, create batches
        batch_size = 1
        train_dataset = tf.data.Dataset.from_tensor_slices((goal_positions[:, 0:2], end_positions[:, 0:2]))
        train_dataset = train_dataset.shuffle(buffer_size=1024).batch(batch_size)

        epochs = 1000
        ppError_avg_list = []
        for epoch in range(last_epoch, epochs):
            ppError_avg = 0
            for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):
                # print(step,x_batch_train,y_batch_train)

                # model.fit(x_batch_train, y_batch_train)
                with tf.GradientTape() as tape:
                    predictions = model(x_batch_train, training=True)
                    ppError = f_ppError(y_batch_train, predictions)
                grads = tape.gradient(ppError, model.trainable_weights)
                optimizer.apply_gradients(zip(grads, model.trainable_weights))
                ppError_avg = ppError_avg + ppError
            ppError_avg = ppError_avg / len(train_dataset)
            ppError_avg_list.append(ppError_avg)
            print(f"Epoch {epoch}: Average ppError: {ppError_avg}")
            if epoch % 5 == 0:
                keras.models.save_model(model, f"calibration_models/model.{epoch}")

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
            model = keras.models.load_model(self.model_name)
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
        model = keras.models.load_model(self.model_name)

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

        inputs = keras.Input(shape=(2), name="end_points")
        x1 = tf.keras.layers.Dense(64, activation="leaky_relu")(inputs)
        x2 = tf.keras.layers.Dense(64, activation="leaky_relu")(x1)
        outputs = tf.keras.layers.Dense(2, name="start_points")(x2)
        model = keras.Model(inputs=inputs, outputs=outputs)
        model.compile(loss="mse", optimizer=tf.keras.optimizers.Adam(learning_rate=0.003))

        # optimizer and per-prediction error
        optimizer = tf.keras.optimizers.SGD(learning_rate=0.001)
        f_ppError = tf.keras.losses.MeanSquaredError()

        # for stochastic gradient descent, create batches
        batch_size = 1
        train_dataset = tf.data.Dataset.from_tensor_slices((start_points_filter, end_points_filter))
        train_dataset = train_dataset.shuffle(buffer_size=1024).batch(batch_size)

        epochs = 21
        ppError_avg_list = []
        for epoch in range(0, epochs):
            ppError_avg = 0
            for step, (x_batch_train, y_batch_train) in enumerate(train_dataset):
                with tf.GradientTape() as tape:
                    predictions = model(x_batch_train, training=True)
                    ppError = f_ppError(y_batch_train, predictions)
                grads = tape.gradient(ppError, model.trainable_weights)
                optimizer.apply_gradients(zip(grads, model.trainable_weights))
                ppError_avg = ppError_avg + ppError
            ppError_avg = ppError_avg / len(train_dataset)
            ppError_avg_list.append(ppError_avg)
            print(f"Epoch {epoch}: Average ppError: {ppError_avg}")
        keras.models.save_model(model, self.model_name + "_inverse")

    @functools.cached_property
    def adjust_navigation_goal(self):
        calib_file = self.model_name + "_inverse"
        try:
            calib_file = rospy.get_param("~calibration_file", calib_file)
        except ConnectionRefusedError as ce:
            pass

        print("Loading from model " + calib_file)
        model = keras.models.load_model(calib_file)
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


# Run calibration
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Soccer calibration")
    parser.add_argument("--collect", help="Collect", action="store_true")
    args = parser.parse_args()

    c = Calibration()
    if args.collect:
        c.obtain_calibration()
    else:
        # c.calibrate()
        # c.view_calibration()
        # c.invert_model()
        c.test_adjust_navigation_transform()
