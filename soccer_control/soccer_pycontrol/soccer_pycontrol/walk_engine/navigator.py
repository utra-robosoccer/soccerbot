import math
import time
from collections import defaultdict
from typing import List, Union

import matplotlib.pyplot as plt
import numpy as np
import scipy
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.error_calc import (
    find_new_vel,
    heading_error,
    position_error,
)
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize
from soccer_pycontrol.walk_engine.walker import Walker

from soccer_common import PID, Transformation

# from torch.distributed.checkpoint import planner



# TODO could make it more modular by passing in pybullet stuff or have it at one layer higher so we can reuse code
# TODO change to trajectory controller
class Navigator:
    def __init__(
        self,
        world: PybulletWorld,
        bez: Bez,
        imu_feedback_enabled: bool = False,
        ball: bool = False,
        record_walking_metrics: bool = True,
        sim: bool = True,
    ):

        self.world = world
        self.bez = bez
        self.imu_feedback_enabled = imu_feedback_enabled

        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, time.time, ball=ball, sim=sim)
        self.walker = Walker(bez, self.foot_step_planner, imu_feedback_enabled=imu_feedback_enabled)

        self.max_vel = 0.1
        self.nav_x_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )  # TODO could also mod if balance is decreasing

        self.nav_yaw_pid = PID(
            Kp=0.2,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-1, 1),
        )
        self.last_ball = [0, 0]
        self.error_tol = 0.03  # in m TODO add as a param and in the ros version

        # joints
        self.left_ankle_index = self.bez.motor_control.motor_names["left_ankle_roll"]
        self.right_ankle_index = self.bez.motor_control.motor_names["right_ankle_roll"]
        # self.walker.torso_index = self.bez.motor_control.body.

        self.record_walking_metrics = record_walking_metrics
        self.walking_data = defaultdict(list)

    def wait(self, steps: int):
        for i in range(steps):
            time.sleep(self.foot_step_planner.DT)

    # TODO could make input a vector
    def walk(self, target_goal: Union[Transformation, List], ball_pixel=[0, 0], ball_mode: bool = False, display_metrics: bool = False):
        if self.walker.enable_walking:
            if isinstance(target_goal, Transformation):
                if ball_mode:
                    self.walk_ball(target_goal, ball_pixel)
                else:
                    self.walk_pose(target_goal)
            elif isinstance(target_goal, list):  # [d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10]
                self.walk_time(target_goal)

        # if self.record_walking_metrics and display_metrics:
        #     self.display_walking_metrics(show_targets=isinstance(target_goal, Transformation))
        # self.ready()

    def walk_pose(self, target_goal: Transformation):
        pose = self.bez.sensors.get_pose()  # can use self.foot_step_planner.trajectory.get_p_world_CoM(t)

        if self.walker.t < 0:
            self.walker.pid.reset_imus()
            self.nav_x_pid.reset()
            self.nav_x_pid.setpoint = target_goal.position[0]

            self.nav_y_pid.reset()
            self.nav_y_pid.setpoint = target_goal.position[1]

            self.nav_yaw_pid.reset()
            self.nav_yaw_pid.setpoint = target_goal.orientation_euler[0]

            dx, dy = find_new_vel(goal_loc=target_goal.position[:2])
            # he = self.heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])
            # dtheta = math.copysign(0.5, he)
            self.foot_step_planner.setup_walk(dx, dy)
            self.walker.t = 0
            # TODO fix and add to a nav could add a funct for pybullet or python
            # TODO could have a balancing mode by default could use the COM
            # TODO for ball could just remove

        if (
            position_error(pose.position[:2], target_goal.position[:2]) > self.error_tol
            or abs(heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])) > self.error_tol
        ):  # self.bez.sensors.get_pose() #TODO about 20% or 40% error
            pose = (
                self.bez.sensors.get_pose()
            )  # self.foot_step_planner.robot.get_T_world_trunk()  # can use self.foot_step_planner.trajectory.get_p_world_CoM(t)

            # TODO should be broken up and a unit test
            # print(self.foot_step_planner.robot.com_world())
            goal = Transformation()
            goal.rotation_matrix = np.matmul(target_goal.rotation_matrix, scipy.linalg.inv(pose.rotation_matrix))
            goal.position = pose.rotation_matrix.T @ target_goal.position - pose.rotation_matrix.T @ pose.position
            # print(goal.position , pose.position)
            x_error = target_goal.position[0] - pose.position[0]
            y_error = target_goal.position[1] - pose.position[1]
            head_error = heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])
            # TODO replace with pure pursuit
            # TODO make  a 2d unit test
            self.nav_x_pid.setpoint = goal.position[0]
            self.nav_y_pid.setpoint = goal.position[1]
            dx = self.nav_x_pid.update(0)
            dy = self.nav_y_pid.update(0)
            dtheta = self.nav_yaw_pid.update(pose.orientation_euler[0])
            print(round(dx, 3), " ", round(dy, 3), " ", round(dtheta, 3), " ", round(x_error, 3), " ", round(y_error, 3), " ", round(head_error, 3))
            self.foot_step_planner.configure_planner(dx, dy, dtheta)
            self.walker.walk_loop()  # TODO move main loop out of here
        else:
            self.ready()
            self.walker.enable_walking = False

    def walk_ball(self, target_goal: Transformation, ball_pixel):
        if self.walker.t < 0:
            self.walker.pid.reset_imus()

            self.nav_x_pid.reset()
            self.nav_x_pid.setpoint = target_goal.position[0]

            self.nav_y_pid.reset()
            self.nav_y_pid.setpoint = target_goal.position[1]

            self.nav_yaw_pid.reset()
            self.nav_yaw_pid.setpoint = 0  # TODO add  yaw modes

            dx, dy = find_new_vel(goal_loc=target_goal.position[:2])
            self.foot_step_planner.setup_walk(dx, dy)
            self.walker.t = 0
            # TODO fix and add to a nav could add a funct for pybullet or python
            # TODO could have a balancing mode by default could use the COM
            # TODO for ball could just remove
        if (
            position_error(target_goal.position[:2]) > self.error_tol
            or abs(heading_error(target_goal.orientation_euler[0], self.bez.sensors.get_pose().orientation_euler[0])) > self.error_tol
        ):
            self.nav_x_pid.setpoint = target_goal.position[0]
            self.nav_y_pid.setpoint = target_goal.position[1]

            # TODO replace with pure pursuit
            # TODO make  a 2d unit test

            dx = self.nav_x_pid.update(0)
            dy = self.nav_y_pid.update(0)

            dtheta = self.nav_yaw_pid.update(self.bez.sensors.get_pose().orientation_euler[0])
            # print(round(dx, 3), " ", round(dy, 3), " ", round(dtheta, 3), " ", round(x_error, 3), " ", round(y_error, 3), " ", round(head_error, 3))
            self.foot_step_planner.configure_planner(dx, dy, dtheta)

            self.walker.walk_loop(target_goal.position, ball_pixel)
        # else:
        #     self.ready()
        #     self.walker.enable_walking = False

    def walk_time(self, target_goal: list):
        if self.walker.t < 0:
            self.foot_step_planner.setup_walk(target_goal[0], target_goal[1], target_goal[2], target_goal[3])
            self.walker.pid.reset_imus()
            self.walker.t = 0

        if self.walker.t < target_goal[4]:
            self.walker.walk_loop()
        elif target_goal[4] <= self.walker.t:
            self.ready()
            self.walker.enable_walking = False

    def ready(self):
        self.foot_step_planner.setup_tasks()

        self.bez.motor_control.set_angles_from_placo(self.foot_step_planner.robot)
        self.bez.motor_control.configuration["left_shoulder_roll"] = 0.1
        self.bez.motor_control.configuration["right_shoulder_roll"] = 0.1

        self.bez.motor_control.set_single_motor("head_yaw", self.walker.ball_dx)
        self.bez.motor_control.set_single_motor("head_pitch", 0.7)
        self.bez.motor_control.set_motor()

    def display_walking_metrics(self, show_targets: bool = False) -> None:
        fig, (ax_imu0, ax_imu1, ax_imu2) = plt.subplots(3, 1, sharex=True)
        fig.canvas.set_window_title("imu")

        imu_0 = np.array(np.array(self.walking_data["IMU_0"]).transpose())
        ax_imu0.plot(imu_0[0, :], imu_0[1, :])
        if show_targets:
            ax_imu0.plot(
                imu_0[0, :],
                np.ones(imu_0[0, :].shape) * self.nav_yaw_pid.setpoint,
                linewidth=0.5,
                color="r",
                label=f"target yaw ({self.nav_yaw_pid.setpoint})",
            )
        ax_imu0.set_title("yaw")
        ax_imu0.grid()

        imu_1 = np.array(np.array(self.walking_data["IMU_1"]).transpose())
        ax_imu1.plot(imu_1[0, :], imu_1[1, :])
        ax_imu1.plot(imu_1[0, :], np.zeros(imu_1[0, :].shape), linewidth=0.5, color="r", label=f"target pitch ({0.0})")
        ax_imu1.set_title("pitch")
        ax_imu1.grid()

        imu_2 = np.array(np.array(self.walking_data["IMU_2"]).transpose())
        ax_imu2.plot(imu_2[0, :], imu_2[1, :])
        ax_imu2.plot(imu_2[0, :], np.zeros(imu_2[0, :].shape), linewidth=0.5, color="r", label=f"target roll ({0.0})")
        ax_imu2.set_title("roll")
        ax_imu2.grid()

        plt.subplots_adjust(wspace=0.3, hspace=0.5)

        plt.show()

        # fig, ax = plt.subplots(3, 1)
        fig = plt.figure(figsize=(5, 7))
        fig.canvas.set_window_title("position")
        gs = fig.add_gridspec(10, 1)

        ax_position = fig.add_subplot(gs[:6])
        ax_pos_err = fig.add_subplot(gs[7])
        ax_yaw_err = fig.add_subplot(gs[9])

        target_x = 0 if not show_targets else self.nav_x_pid.setpoint
        target_y = 0 if not show_targets else self.nav_y_pid.setpoint
        target_yaw = 0 if not show_targets else self.nav_yaw_pid.setpoint

        position = np.array(self.walking_data["POSITION"]).transpose()
        ax_position.plot(position[1, :], position[2, :])
        ax_position.plot(position[1, 0], position[2, 0], "yo", label="start point")
        ax_position.plot(position[1, -1], position[2, -1], "go", label="end point")
        if show_targets:
            ax_position.plot(self.nav_x_pid.setpoint, self.nav_y_pid.setpoint, "ro", label="target point")
        ax_position.set_title("position")
        ax_position.set_xlabel("x")
        ax_position.set_ylabel("y")
        ax_position.grid()
        ax_position.legend()

        if show_targets:
            ax_pos_err.plot(position[0, :], np.linalg.norm(position[1:3, :].transpose() - np.array([target_x, target_y]), axis=1))
            ax_pos_err.plot(position[0, :], np.zeros(position[0, :].shape), linewidth=0.5, color="r")
            ax_pos_err.set_title("position error")
            ax_pos_err.set_ylabel("euclidean distance")
            ax_pos_err.grid()

            ax_yaw_err.plot(position[0, :], position[3, :] - target_yaw)
            ax_yaw_err.plot(position[0, :], np.zeros(position[0, :].shape), linewidth=0.5, color="r")
            ax_yaw_err.set_title("orientation error")
            ax_yaw_err.grid()

        plt.show()

        fig = plt.figure()
        fig.canvas.set_window_title("3d position")
        gs = fig.add_gridspec(1, 1)

        ax_path = fig.add_subplot(gs[0], projection="3d")

        left = np.array(self.walking_data["LEFT_FOOT"]).T
        right = np.array(self.walking_data["RIGHT_FOOT"]).T
        com = np.array(self.walking_data["COM"]).T

        ax_path.plot(left[1, :], left[2, :], left[3, :], label="left foot")
        ax_path.plot(right[1, :], right[2, :], right[3, :], label="right foot")
        ax_path.plot(com[1, :], com[2, :], com[3, :], label="centre of mass")
        # ax_path.plot(com[1, :], com[2, :], np.zeros(com[3, :].shape), label="centre of mass projected")
        ax_path.set_title("robot stance")
        ax_path.grid()
        ax_path.legend()

        plt.show()

        fig = plt.figure()
        fig.canvas.set_window_title("footprints")
        gs = fig.add_gridspec(1, 1)

        ax_footprints = fig.add_subplot(gs[0])

        # both feet on the ground
        ground = 0.025
        left_grounded_i = np.where(left[3] < ground)[0]
        right_grounded_i = np.where(right[3] < ground)[0]

        rectangle_width = 0.1
        rectangle_length = 0.05
        rectangle_x_offset = 0
        rectangle_y_offset = 0

        make_rotation = lambda theta, x, y: np.array([[np.cos(theta), -np.sin(theta), x], [np.sin(theta), np.cos(theta), y]])
        rectangle = np.array(
            [
                [rectangle_x_offset - rectangle_width / 2, rectangle_y_offset + rectangle_length / 2, 1],
                [rectangle_x_offset + rectangle_width / 2, rectangle_y_offset + rectangle_length / 2, 1],
                [rectangle_x_offset + rectangle_width / 2, rectangle_y_offset - rectangle_length / 2, 1],
                [rectangle_x_offset - rectangle_width / 2, rectangle_y_offset - rectangle_length / 2, 1],
                [rectangle_x_offset - rectangle_width / 2, rectangle_y_offset + rectangle_length / 2, 1],
            ]
        )
        make_rectangle = lambda theta, x, y: np.array([make_rotation(theta, x, y) @ rectangle[i].T for i in range(5)])

        yaw_data = np.array(self.walking_data["IMU_0"]).T

        left_rectangles = []
        for i in left_grounded_i:
            if np.any(np.where(yaw_data[0] == left[0, i])[0]):
                left_rectangles.append(make_rectangle(yaw_data[1][np.where(yaw_data[0] == left[0, i])[0][0]], left[1, i], left[2, i]).T)

        right_rectangles = []
        for i in right_grounded_i:
            if np.any(np.where(yaw_data[0] == right[0, i])[0]):
                right_rectangles.append(make_rectangle(yaw_data[1][np.where(yaw_data[0] == right[0, i])[0][0]], right[1, i], right[2, i]).T)

        for r in left_rectangles:
            ax_footprints.plot(r[0], r[1])
        for r in right_rectangles:
            ax_footprints.plot(r[0], r[1])

        ax_footprints.plot(com[1, :], com[2, :], label="centre of mass")

        ax_footprints.set_aspect("equal")

        plt.show()

    def clear_walking_metrics(self, target_data: list = None) -> None:
        """reinitialize walking data"""
        if not target_data:
            # clear all
            self.walking_data = defaultdict(list)
            return
        for name in self.walking_data.keys():
            if name in target_data:
                self.walking_data[name] = []

    def update_walking_metrics(self, t: float) -> None:
        """update stored data for time t"""

        # IMU data
        imu_data = self.bez.sensors.get_imu()
        self.walking_data["IMU_0"].append((t, imu_data[0]))
        self.walking_data["IMU_1"].append((t, imu_data[1]))
        self.walking_data["IMU_2"].append((t, imu_data[2]))

        # position data
        pose = self.bez.sensors.get_pose()
        self.walking_data["POSITION"].append((t, pose.position[0], pose.position[1], pose.orientation_euler[0]))

        # feet and COM data
        left_foot = self.bez.sensors.get_pose(self.left_ankle_index)  # 13 for left foot joint
        right_foot = self.bez.sensors.get_pose(self.right_ankle_index)  # 7 for right foot joint
        com = self.bez.sensors.get_pose(1)  # 1 for torso joint
        self.walking_data["LEFT_FOOT"].append((t, left_foot.position[0], left_foot.position[1], left_foot.position[2]))
        self.walking_data["RIGHT_FOOT"].append((t, right_foot.position[0], right_foot.position[1], right_foot.position[2]))
        self.walking_data["COM"].append((t, com.position[0], com.position[1], com.position[2]))


if __name__ == "__main__":
    world = PybulletWorld(
        camera_yaw=90,
        real_time=True,
        rate=200,
    )
    bez = Bez(robot_model="bez1")
    walk = Navigator(world, bez)
    walk.walk(t_goal=100)
