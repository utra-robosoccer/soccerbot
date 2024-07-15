import numpy as np
import pybullet as pb
import scipy
from soccer_pycontrol.common.links import Links
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common.transformation import Transformation


class WalkEngine:
    """
    Main loop for the walk engine. Interfaces with all classes to perform walking.
    """

    #

    def __init__(self, world: PybulletWorld, bez: Bez):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """

        self.world = world
        self.bez = bez
        self.PYBULLET_STEP = 0.01

        self.step_planner = FootStepPlanner(
            sim="_sim",
            robot_model=self.bez.robot_model,
            walking_torso_height=self.bez.data.walking_torso_height,
            foot_center_to_floor=self.bez.data.foot_center_to_floor,
            torso_offset_pitch=self.bez.data.torso_offset_pitch,
            torso_offset_x=self.bez.data.torso_offset_x,
        )
        self.pid = Stabilize()

        self.terminate_walk = False
        self.prepare_walk_time = 2
        # TODO should this be an input?
        self.t = 0

    def set_goal(self, goal: Transformation, transform_global: bool = True) -> None:
        """
        Set the goal of the robot, will create the path to the goal that will be executed in the run() loop

        :param goal: The 3D location goal for the robot
        """
        if transform_global:
            goal = self.transform_global_local(goal)

        self.step_planner.create_path_to_goal(goal)

    def transform_global_local(self, goal: Transformation) -> Transformation:
        current_pose = self.bez.sensors.get_pose()
        goal.rotation_matrix = np.matmul(goal.rotation_matrix, scipy.linalg.inv(current_pose.rotation_matrix))
        goal.position = current_pose.rotation_matrix.T @ goal.position - current_pose.rotation_matrix.T @ current_pose.position
        return goal

    def wait(self, step: int) -> None:
        self.world.wait(step)

    def ready(self) -> None:
        self.bez.ready()

    def walk(self) -> bool:
        """
        The main run loop for the navigator, executes goals given through setGoal and then stops

        :return: True if the robot succeeds navigating to the goal, False if it doesn't reach the goal and falls
        """

        self.t = -self.prepare_walk_time
        stable_count = 20
        self.pid.reset_imus()

        while True:
            [_, pitch, roll] = self.bez.sensors.get_euler_angles()
            # TODO should we have more abstraction, because this is actually useful to know where the functions go
            if 0 <= self.t <= self.step_planner.robot_path.duration():
                # TODO after add metrics for evaluating walking come back see if this is necessary
                if self.step_planner.current_step_time <= self.t <= self.step_planner.robot_path.duration():
                    torso_to_right_foot, torso_to_left_foot = self.step_planner.get_next_step(self.t)

                    self.bez.find_joint_angles(torso_to_right_foot, torso_to_left_foot)

                    self.stabilize_walk(pitch, roll)

                    self.bez.motor_control.set_motor()
                    self.step_planner.current_step_time = self.step_planner.current_step_time + self.step_planner.robot_path.step_precision
            else:
                # print(stable_count)
                stable_count = self.update_stable_count(pitch, roll, stable_count)
                if stable_count < 0:  # TODO dont really like this format
                    break  # TODO this is bad

                self.stabilize_stand(pitch, roll)

            if self.bez.fallen(pitch):
                return False

            self.world.step()
            self.t = self.t + self.PYBULLET_STEP
        return True

    def stabilize_stand(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.standing_pitch_pid.update(pitch)
        # self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)
        self.bez.motor_control.set_leg_joint_5_target_angle(error_pitch)

        # error_roll = self.pid.standing_roll_pid.update(roll)
        # self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)

        # self.bez.motor_control.set_motor()

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)

        # error_roll = self.pid.walking_roll_pid.update(roll)
        # self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)

        # self.bez.motor_control.set_motor()

    def update_stable_count(self, pitch: float, roll: float, stable_count: int) -> int:
        # TODO tune threshhold
        if abs(pitch - self.pid.standing_pitch_pid.setpoint) < 0.06 and abs(roll - self.pid.standing_roll_pid.setpoint) < 0.06:
            stable_count -= 1
            if stable_count == 0:
                if self.t < 0:
                    self.t = 0
                    stable_count = 20
                else:
                    stable_count = -1
        else:
            stable_count = 5
        return stable_count
