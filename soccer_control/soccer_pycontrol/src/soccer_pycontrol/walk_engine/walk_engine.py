import time
from typing import List

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize


class WalkEngine:
    def __init__(self, world: PybulletWorld, bez: Bez, imu_feedback_enabled: bool = True):
        self.world = world
        self.bez = bez
        self.imu_feedback_enabled = imu_feedback_enabled
        self.func_step = self.world.step
        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, time.time)

        self.pid = Stabilize(self.bez.parameters)

    # TODO need way to have no imu maybe unit tests
    def walk(self, d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10):
        self.foot_step_planner.setup_walk(d_x, d_y, d_theta, nb_steps)
        self.pid.reset_imus()
        t = 0
        while t < t_goal:
            self.foot_step_planner.walk_loop(t)

            self.bez.motor_control.configuration = self.filter_joints()

            if self.imu_feedback_enabled and self.bez.sensors.imu_ready:
                [_, pitch, roll] = self.bez.sensors.get_imu()
                self.stabilize_walk(pitch, roll)

            self.bez.motor_control.set_motor()
            self.func_step()

            t = self.foot_step_planner.step(t)

    def ready(self):
        self.foot_step_planner.setup_tasks()
        self.bez.motor_control.configuration = self.filter_joints()

        self.bez.motor_control.set_motor()

    def wait(self, step: int) -> None:
        self.world.wait(step)

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)

        error_roll = self.pid.walking_roll_pid.update(roll)
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
    walk = WalkEngine(world, bez)
    walk.walk(t_goal=100)
