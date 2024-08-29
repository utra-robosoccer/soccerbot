import unittest

import numpy as np
import pybullet as pb
import pytest
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld

from soccer_common import Transformation

REAL_TIME = False


class TestPybullet(unittest.TestCase):
    def tearDown(self):
        self.world.close()
        del self.bez
        del self.world

    def test_imu(self):
        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        # pose = Transformation()
        pose = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
        pose = Transformation(position=[0, 0, 0.070], euler=[0, 1.57, 0])
        self.bez = Bez(robot_model="bez2", pose=pose)
        self.world.wait(100)
        for i in range(100):
            [_, pitch, roll] = self.bez.sensors.get_imu()
            print(self.bez.fallen(pitch))
            self.world.step()
        # TODO add more

    def test_foot_sensor(self):
        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        self.bez = Bez(robot_model="bez2", pose=Transformation())
        self.world.wait(100)
        print(self.bez.sensors.get_foot_pressure_sensors(self.world.plane))
        # TODO add more and fuix the link location

    def test_bez_motor_range(self):
        self.world = PybulletWorld(path="", camera_yaw=90, real_time=REAL_TIME, rate=1000)
        self.bez = Bez(robot_model="bez2", fixed_base=True, pose=Transformation())
        self.world.wait(50)
        angles = np.linspace(-np.pi, np.pi)
        for i in range(self.bez.motor_control.numb_of_motors):

            for j in angles:
                x = [0.0] * self.bez.motor_control.numb_of_motors
                x[i] = j

                pb.setJointMotorControlArray(
                    bodyIndex=self.bez.model.body,
                    controlMode=pb.POSITION_CONTROL,
                    jointIndices=list(range(0, self.bez.motor_control.numb_of_motors, 1)),
                    targetPositions=x,
                )
                self.world.wait_motor()

        self.world.wait(100)

    def test_bez_motor_range_single(self):
        self.world = PybulletWorld(path="", camera_yaw=90, real_time=REAL_TIME, rate=500)
        self.bez = Bez(robot_model="bez1", fixed_base=True, pose=Transformation())
        self.world.wait(50)
        angles = np.linspace(-np.pi, np.pi)

        for j in angles:
            x = [0.0] * self.bez.motor_control.numb_of_motors
            x[self.bez.motor_control.motor_names.index("left_ankle_roll")] = j
            x[self.bez.motor_control.motor_names.index("right_ankle_roll")] = j

            pb.setJointMotorControlArray(
                bodyIndex=self.bez.model.body,
                controlMode=pb.POSITION_CONTROL,
                jointIndices=list(range(0, self.bez.motor_control.numb_of_motors, 1)),
                targetPositions=x,
            )
            self.world.wait_motor()

        self.world.wait(100)

    # TODO fix or wait for mujoco?
    # def test_stand_plane(self):
    #     self.world = PybulletWorld(camera_yaw=0, real_time=REAL_TIME, rate=100)
    #     self.bez = Bez(pose=Transformation())
    #     tf = Navigator(self.world, self.bez)
    #     tf.wait(50)
    #     tf.bez.ready()
    #     tf.wait(50)
    #     while tf.t < 1000:
    #         [_, pitch, roll] = tf.bez.sensors.get_imu()
    #         # if tf.fallen(pitch):
    #         #     pb.applyExternalForce(p.model.body, Links.TORSO, [0, 5, 0], [0, 0, 0], pb.LINK_FRAME)
    #         # p.model.set_pose()
    #         # TODO fix link alignment
    #         # pb.applyExternalForce(tf.bez.model.body, Links.TORSO, [3, 0, 0], [0, 0, 0], pb.LINK_FRAME)
    #         tf.stabilize_stand(pitch, roll)
    #         tf.world.step()
    #         tf.t += +0.01
    #     tf.world.wait(100)


# @pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
# @pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
# @pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
# def test_head_localizing_sweep(robot_model: str):
#     """
#     Case 1: Standard case
#     :return: None
#     """
#     self.world = PybulletWorld(path="", camera_yaw=90, real_time=REAL_TIME, rate=1000, cameraTargetPosition=[0, 0, 0.25])
#     self.bez = Bez(robot_model=robot_model, fixed_base=True, pose=Transformation())
#     steps = 200
#     thetas = self.bez.ik_actions.head_localizing_sweep()
#     head_step = 0
#     for i in range(steps):
#         self.bez.motor_control.set_head_target_angles(thetas[i][:])
#         self.bez.motor_control.set_motor()
#         self.world.wait_motor()
#         head_step += 1
#
#     self.world.wait(steps)
#
#     self.world.close()


#
# @pytest.mark.parametrize("sweep_name", ["x", "y", "z"])
# @pytest.mark.parametrize("h", [0.0, 0.05, 0.1])
# @pytest.mark.parametrize("robot_model", ["bez1", "bez2"])
# def test_sweep(sweep_name: str, h: float, robot_model: str):
#     """
#     Case 1: Standard case
#     :return: None
#     """
#
#     self.world = PybulletWorld(path="", camera_yaw=45, real_time=REAL_TIME, rate=1000, cameraTargetPosition=[0, 0, 0.25])
#     self.bez = Bez(robot_model=robot_model, fixed_base=True, pose=Transformation())
#     steps = 50
#     x = np.zeros(steps)
#
#     def sweep(target_pose: np.ndarray, step: int) -> None:
#         for i in range(step):
#             self.bez.motor_control.set_right_leg_target_angles(target_pose[i][0:6])
#             self.bez.motor_control.set_motor()
#             self.world.wait_motor()
#
#     if sweep_name == "x":
#         x, _, _ = self.bez.ik_actions.x_sweep(h)
#     elif sweep_name == "y":
#         x, _, _ = self.bez.ik_actions.y_sweep(h)
#     elif sweep_name == "z":
#         x, _, _ = self.bez.ik_actions.z_sweep()
#
#     sweep(x, steps)
#     self.world.wait(steps)
#
#     self.world.close()
