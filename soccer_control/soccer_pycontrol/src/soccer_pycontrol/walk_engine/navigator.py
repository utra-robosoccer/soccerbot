import math
import time
from typing import List, Union

import numpy as np
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common import PID, Transformation


# TODO change to trajectory controller
class Navigator:
    def __init__(self, world: PybulletWorld, bez: Bez, imu_feedback_enabled: bool = False):
        self.world = world
        self.bez = bez
        self.imu_feedback_enabled = imu_feedback_enabled
        self.func_step = self.world.step
        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, time.time)

        self.walk_pid = Stabilize(self.bez.parameters)
        self.max_vel = 0.09
        self.nav_x_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.1,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel + 0.01, self.max_vel),
        )  # TODO could also mod if balance is decreasing

        self.nav_yaw_pid = PID(
            Kp=0.1,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-1, 1),
        )

        self.error_tol = 0.01  # in m TODO add as a param and in the ros version

    # TODO could make input a vector
    def walk(self, target_goal: Union[Transformation, List], ball_mode: bool = False):
        if isinstance(target_goal, Transformation):
            if ball_mode:
                self.walk_ball(target_goal)
            else:
                self.walk_pose(target_goal)
        elif isinstance(target_goal, list):  # [d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10]
            self.walk_time(target_goal)

        self.ready()

    def find_new_vel(self, goal_loc: list, curr_loc: list = (0, 0)):
        x_error = abs(goal_loc[0] - curr_loc[0])
        y_error = abs(goal_loc[1] - curr_loc[1])

        if x_error > y_error:
            dx = self.max_vel
            dy = dx * (y_error / x_error)
        elif x_error < y_error:
            dy = self.max_vel
            dx = dy * (x_error / y_error)
        else:
            dx = self.max_vel
            dy = self.max_vel

        return math.copysign(dx, goal_loc[0]), math.copysign(dy, goal_loc[1])

    def walk_pose(self, target_goal: Transformation):
        self.walk_pid.reset_imus()
        self.nav_x_pid.reset()
        self.nav_x_pid.setpoint = target_goal.position[0]

        self.nav_y_pid.reset()
        self.nav_y_pid.setpoint = target_goal.position[1]

        self.nav_yaw_pid.reset()
        self.nav_yaw_pid.setpoint = target_goal.orientation_euler[0]

        pose = self.bez.sensors.get_pose()  # can use self.foot_step_planner.trajectory.get_p_world_CoM(t)
        dx, dy = self.find_new_vel(goal_loc=target_goal.position[:2])
        # he = self.heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])
        # dtheta = math.copysign(0.5, he)
        self.foot_step_planner.setup_walk(dx, dy)
        t = 0
        # TODO fix and add to a nav could add a funct for pybullet or python
        # TODO could have a balancing mode by default could use the COM
        # TODO for ball could just remove

        while (
            self.position_error(pose.position[:2], target_goal.position[:2]) > self.error_tol
            or abs(self.heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])) > self.error_tol
        ):  # self.bez.sensors.get_pose() #TODO about 20% or 40% error
            pose = (
                self.bez.sensors.get_pose()
            )  # self.foot_step_planner.robot.get_T_world_trunk()  # can use self.foot_step_planner.trajectory.get_p_world_CoM(t)
            x_error = target_goal.position[0] - pose.position[0]
            y_error = target_goal.position[1] - pose.position[1]
            head_error = self.heading_error(target_goal.orientation_euler[0], pose.orientation_euler[0])
            # TODO replace with pure pursuit
            # TODO make  a 2d unit test

            dx = self.nav_x_pid.update(pose.position[0])
            dy = self.nav_y_pid.update(pose.position[1])
            dtheta = self.nav_yaw_pid.update(pose.orientation_euler[0])
            print(round(dx, 3), " ", round(dy, 3), " ", round(dtheta, 3), " ", round(x_error, 3), " ", round(y_error, 3), " ", round(head_error, 3))
            self.foot_step_planner.configure_planner(dx, dy, dtheta)

            # if t > 5:
            #
            #     target_goal = Transformation(position=[-0.5,-0.5,0])
            #     dx, dy = self.find_new_vel(goal_loc=target_goal.position[:2], curr_loc=position[:2])
            #     self.foot_step_planner.configure_planner(dx, dy)

            t = self.walk_loop(t)

    def walk_ball(self, target_goal: Transformation):
        self.walk_pid.reset_imus()
        self.nav_x_pid.reset()
        self.nav_x_pid.setpoint = target_goal.position[0]

        self.nav_y_pid.reset()
        self.nav_y_pid.setpoint = target_goal.position[1]

        self.nav_yaw_pid.reset()
        self.nav_yaw_pid.setpoint = 0

        dx, dy = self.find_new_vel(goal_loc=target_goal.position[:2])
        # dx, dy = 0, 0
        self.foot_step_planner.setup_walk(dx, dy)
        t = 0
        # TODO fix and add to a nav could add a funct for pybullet or python
        # TODO could have a balancing mode by default could use the COM
        # TODO for ball could just remove
        while (
            self.position_error(target_goal.position[:2]) > self.error_tol
            or abs(self.heading_error(target_goal.orientation_euler[0], self.bez.sensors.get_pose().orientation_euler[0])) > self.error_tol
        ):
            target_goal = self.bez.sensors.get_ball()

            # self.foot_step_planner.head_movement(target_goal.position)

            self.nav_x_pid.setpoint = target_goal.position[0]
            self.nav_y_pid.setpoint = target_goal.position[1]
            x_error = target_goal.position[0]
            y_error = target_goal.position[1]
            head_error = self.heading_error(target_goal.orientation_euler[0], self.bez.sensors.get_pose().orientation_euler[0])
            # TODO replace with pure pursuit
            # TODO make  a 2d unit test

            dx = self.nav_x_pid.update(0)
            dy = self.nav_y_pid.update(0)

            dtheta = self.nav_yaw_pid.update(self.bez.sensors.get_pose().orientation_euler[0])
            print(round(dx, 3), " ", round(dy, 3), " ", round(dtheta, 3), " ", round(x_error, 3), " ", round(y_error, 3), " ", round(head_error, 3))
            self.foot_step_planner.configure_planner(dx, dy, dtheta)

            t = self.walk_loop(t)

    def walk_time(self, target_goal: list):
        self.foot_step_planner.setup_walk(target_goal[0], target_goal[1], target_goal[2], target_goal[3])
        self.walk_pid.reset_imus()
        t = 0
        while t < target_goal[4]:
            self.foot_step_planner.head_movement(self.bez.sensors.get_ball().position)
            t = self.walk_loop(t)

    def walk_loop(self, t: float):
        self.foot_step_planner.plan_steps(t)
        self.bez.motor_control.configuration = self.filter_joints()

        if self.imu_feedback_enabled and self.bez.sensors.imu_ready:
            [_, pitch, roll] = self.bez.sensors.get_imu()
            self.stabilize_walk(pitch, roll)

        self.bez.motor_control.set_motor()
        self.func_step()

        t = self.foot_step_planner.step(t)
        return t

    @staticmethod
    def heading_error(theta_desired: float, theta_current: float) -> float:
        """
        Calculates the position and orientation error between a current pose and a desired pose.

        :return: A tuple of (position_error, desired_yaw, heading_error) representing the errors.
        """

        head_error = theta_desired - theta_current

        # Normalize the heading error to be between -pi and pi.
        heading_error_norm = math.atan2(math.sin(head_error), math.cos(head_error))

        return heading_error_norm

    # @staticmethod
    # def desired_yaw(self) -> float:
    #     # Calculate the desired yaw (angle of rotation) using the arctan2 function.
    #     numerator = self.goal_loc[1] - self.curr_loc[1]
    #     denominator = self.goal_loc[0] - self.curr_loc[0]
    #     return float(np.arctan2(numerator, denominator))

    @staticmethod
    def position_error(goal_loc: np.ndarray, curr_loc: np.ndarray = (0, 0)):
        return float(np.linalg.norm(goal_loc - curr_loc))

    def ready(self):
        self.foot_step_planner.setup_tasks()
        self.bez.motor_control.configuration = self.filter_joints()

        self.bez.motor_control.set_motor()

    def wait(self, step: int) -> None:
        self.world.wait(step)

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.walk_pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)  # TODO retune
        pass
        error_roll = self.walk_pid.walking_pitch_pid.update(roll)  # TODO retune
        self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)

    def filter_joints(self) -> List[int]:
        joints = [0] * self.bez.motor_control.numb_of_motors
        for joint in self.bez.motor_control.motor_names:
            joints[self.bez.motor_control.motor_names.index(joint)] = self.foot_step_planner.robot.get_joint(joint)
        return joints


if __name__ == "__main__":
    world = PybulletWorld(
        camera_yaw=90,
        real_time=True,
        rate=200,
    )
    bez = Bez(robot_model="bez1")
    walk = Navigator(world, bez)
    walk.walk(t_goal=100)
