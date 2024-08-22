import time

from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.stabilize import Stabilize
from soccer_pycontrol.walk_engine.walk_placo import WalkPlaco


class WalkEnginePlaco:
    def __init__(self, world: PybulletWorld, bez: Bez):
        self.world = world
        self.bez = bez

        self.walk_placo = WalkPlaco(bez.robot_model, time.time)

        self.pid = Stabilize(
            sim="_sim",
            robot_model=self.bez.robot_model,
        )

    def walk(self, d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10):
        self.walk_placo.setup_walk(d_x, d_y, d_theta, nb_steps)
        self.pid.reset_imus()
        t = 0
        while t < t_goal:  # TODO needs end condition
            [_, pitch, roll] = self.bez.sensors.get_euler_angles()

            self.walk_placo.walk_loop(t)

            joints = [0] * self.bez.motor_control.numb_of_motors
            for joint in self.bez.motor_control.motor_names:
                if joint in ["head_base_frame", "trunk_frame", "camera_frame", "left_foot_frame", "right_foot_frame", "/torso_imu"]:
                    continue
                joints[self.bez.motor_control.motor_names.index(joint)] = self.walk_placo.robot.get_joint(joint)

            self.bez.motor_control.configuration = joints
            self.stabilize_walk(pitch, roll)

            self.bez.motor_control.set_motor()
            self.world.step()

            t = self.walk_placo.step(t)

    def wait(self, step: int) -> None:
        self.world.wait(step)

    def ready(self) -> None:
        self.bez.ready()

    def stabilize_walk(self, pitch: float, roll: float) -> None:
        error_pitch = self.pid.walking_pitch_pid.update(pitch)
        self.bez.motor_control.set_leg_joint_3_target_angle(error_pitch)
        pass
        error_roll = self.pid.walking_roll_pid.update(roll)
        self.bez.motor_control.set_leg_joint_2_target_angle(error_roll)
        # print(pitch, roll, error_roll)
        # self.bez.motor_control.set_motor()


if __name__ == "__main__":
    world = PybulletWorld(
        camera_yaw=90,
        real_time=True,
        rate=200,
    )
    bez = Bez(robot_model="bez1")
    walk = WalkEnginePlaco(world, bez)
    walk.walk(t_goal=100)
