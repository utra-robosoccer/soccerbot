from typing import List

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common import PID, Transformation


# TODO could make it more modular by passing in pybullet stuff or have it at one layer higher so we can reuse code
# TODO change to trajectory controller
class Walker:
    def __init__(self, bez: Bez, planner: FootStepPlanner, imu_feedback_enabled: bool = False):
        self.ball_dx = 0
        self.ball_dy = 0.7
        self.bez = bez
        self.imu_feedback_enabled = imu_feedback_enabled
        self.foot_step_planner = planner
        self.pid = Stabilize(self.bez.parameters)
        self.last_ball = [0, 0]
        self.t = None
        self.enable_walking = None
        self.reset_walk()
        self.ball_x_pid = PID(
            Kp=0.05,
            Kd=0,
            Ki=0.03,
            setpoint=0,
            output_limits=(-1.57, 1.57),
        )

        self.ball_y_pid = PID(
            Kp=-0.05,
            Kd=0,
            Ki=-0.03,
            setpoint=2.4,
            output_limits=(0.4, 1.3),
        )

    def reset_walk(self):
        self.t = -1
        self.enable_walking = True

    # TODO could make input a vector

    def walk_loop(self, target_goal=[0, 0], ball_pixel=[0, 0]):  # TODO set default to something better
        self.foot_step_planner.plan_steps(self.t)
        self.bez.motor_control.set_angles_from_placo(self.foot_step_planner.robot)

        if self.imu_feedback_enabled and self.bez.sensors.imu_ready:
            [_, pitch, roll] = self.bez.sensors.get_imu()
            # print(pitch,"  ", roll)
            self.stabilize_walk(pitch, roll)

        # self.foot_step_planner.head_movement(target_goal)
        if ball_pixel != self.last_ball:
            # self.foot_step_planner.head_movement(target_goal.position)

            self.last_ball = ball_pixel
            self.ball_dx = self.ball_x_pid.update(3.2 - ball_pixel[0] / 100.0)
            self.ball_dy = self.ball_y_pid.update(ball_pixel[1] / 100.0)
        # print(f"{ball_pixel}, {self.ball_dx}, {self.ball_dy}")
        self.bez.motor_control.configuration["head_yaw"] = self.ball_dx
        self.bez.motor_control.configuration["head_pitch"] = self.ball_dy

        # self.bez.motor_control.configuration["head_yaw"] = self.ball_dx
        # self.bez.motor_control.configuration["head_pitch"] = 0.7
        # self.bez.motor_control.configuration_offset["left_hip_pitch"] = 0.15
        # self.bez.motor_control.configuration_offset["right_hip_pitch"] = 0.15
        self.bez.motor_control.configuration["left_elbow"] = 1.57
        self.bez.motor_control.configuration["right_elbow"] = 1.57
        self.bez.motor_control.configuration["left_shoulder_roll"] = 0.1
        self.bez.motor_control.configuration["right_shoulder_roll"] = 0.1
        # self.bez.motor_control.configuration["head_pitch"] = 0.7
        # self.bez.motor_control.set_single_motor("head_yaw", 0.7)
        # self.bez.motor_control.set_right_leg_target_angles(
        #     [-0.04679783928424319,
        #      0.08362236855709071,
        #      0.6283185307179586,
        #      -1.3172440991974792,
        #      0.7664105154911365,
        #      -0.1051033439662521]
        # )
        # self.bez.motor_control.set_left_leg_target_angles(
        #     [-0.04679783928424319,
        #      0.08362236855709071,
        #      0.6283185307179586,
        #      -1.3172440991974792,
        #      0.7664105154911365,
        #      -0.1051033439662521]
        # )
        # self.bez.motor_control.set_single_motor("head_pitch", 0.99)
        # self.bez.motor_control.set_right_leg_target_angles([0, 0, 0, 0, 0, 0])
        # self.bez.motor_control.set_left_leg_target_angles([0, 0, 0, 0, 0, 0])
        # self.bez.motor_control.set_head_target_angles(
        #     [0, 0])
        self.bez.motor_control.set_motor()

        self.t = self.foot_step_planner.step(self.t)

        # update joints in control # TODO investigate it has an effect but not sure how much also with step time
        # for joint in self.bez.motor_control.motor_names:
        #     self.foot_step_planner.robot.set_joint(joint,
        #                                            self.bez.motor_control.configuration
        #                                            [self.bez.motor_control.motor_names.index(joint)])
        # T = self.foot_step_planner.robot.get_T_world_fbase()
        # T[0:3,0:3] = self.bez.sensors.get_pose().rotation_matrix
        # self.foot_step_planner.robot.set_T_world_fbase(T)
        # self.foot_step_planner.robot.update_kinematics()

        # if self.record_walking_metrics:
        #     self.update_walking_metrics(t)

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)  # TODO retune

        error_roll = self.pid.walking_roll_pid.update(roll)  # TODO retune
        self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)
