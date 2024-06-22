import pybullet as pb
from soccer_pycontrol.common.links import Links
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common.transformation import Transformation


class WalkEngine:
    """
    Main loop for the walk engine. Interfaces with all classes to perform walking.
    """

    # PYBULLET_STEP = rospy.get_param("control_frequency", 0.01)

    def __init__(self, world: PybulletWorld, bez: Bez):
        """
        Initialize the Navigator

        :param display: Whether or not to show the pybullet visualization, turned off for quick unit tests
        :param useCalibration: Whether or not to use movement calibration files located in config/robot_model.yaml, which adjusts the calibration to the movement given
        """

        self.world = world
        self.bez = bez

        self.step_planner = FootStepPlanner(
            walking_torso_height=self.bez.data.walking_torso_height, foot_center_to_floor=self.bez.data.foot_center_to_floor
        )
        self.pid = Stabilize()

        self.terminate_walk = False
        self.prepare_walk_time = 2  # rospy.get_param("prepare_walk_time", 2)

        self.t = 0

    def set_goal(self, goal: Transformation) -> None:
        """
        Set the goal of the robot, will create the path to the goal that will be executed in the run() loop

        :param goal: The 3D location goal for the robot
        """
        self.step_planner.create_path_to_goal(goal)

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

                    self.step_planner.current_step_time = self.step_planner.current_step_time + self.step_planner.robot_path.step_precision
            else:
                stable_count = self.update_stable_count(pitch, roll, stable_count)
                if stable_count < 0:  # TODO dont really like this format
                    break

                self.stabilize_stand(pitch, roll)

            if self.bez.fallen(pitch):
                return False

            self.world.step()
            self.t = self.t + 0.01
        return True

    def stabilize_stand(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.standing_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)
        print(error_pitch)

        error_roll = self.pid.standing_roll_pid.update(roll)
        self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)

        self.bez.motor_control.set_motor()

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)

        error_roll = self.pid.walking_roll_pid.update(roll)
        self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)

        self.bez.motor_control.set_motor()

    def update_stable_count(self, pitch: float, roll: float, stable_count: int) -> int:
        if abs(pitch - self.pid.standing_pitch_pid.setpoint) < 0.025 and abs(roll - self.pid.standing_roll_pid.setpoint) < 0.025:
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
