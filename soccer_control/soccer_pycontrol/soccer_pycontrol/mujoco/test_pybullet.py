import math
import time
import unittest
from os.path import expanduser

import cv2
import numpy as np
import pytest
from soccer_pycontrol.mujoco.bez import Bez
from soccer_pycontrol.mujoco.simulator import SimWorld

from soccer_common import Transformation
from soccer_pycontrol.walk_engine.navigator import Navigator

REAL_TIME = True


class TestPybullet(unittest.TestCase):
    # def tearDown(self):
    #     self.world.close()
    #     del self.bez
    #     del self.world

    def test_imu(self):
        sim = SimWorld(scene_name="scene_bez2.xml", position="fallen_front")
        bez = Bez(sim)
        start = time.time()

        while sim.t < 200:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            [_, pitch, roll] = bez.sensors.get_imu()
            bez.motor_control.set_single_motor("right_elbow", 1)
            # bez.motor_control.set_right_arm_target_angles([1,1,1])
            # bez.motor_control.set_left_arm_target_angles([1, 1, 1])
            bez.motor_control.set_motor()
            print(bez.motor_control.configuration)
            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_bez_motor_range_single(self):
        sim = SimWorld(scene_name="scene_bez2.xml", position="float")
        bez = Bez(sim)

        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()
        name = "head_pitch"
        # name = "left_hip_roll"

        self.motor_range(angles, name, sim, bez, sim.T)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")
        sim.close_viewer()

    def test_bez_motor_range(self):
        sim = SimWorld(scene_name="scene_bez2.xml",  position="float")
        bez = Bez(sim)

        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()

        for name in bez.motor_control.dof_names:
            bez.motor_control.clear_motor_target()
            bez.motor_control.reset_motors(np.zeros_like(bez.motor_control.configuration))
            sim.render(True)
            sim.step()
            print(name)
            self.motor_range(angles, name, sim, bez, sim.T)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def motor_range(self, angles: np.ndarray, name: str, sim:SimWorld, bez: Bez, T):
        angle_idx = 0
        while angles[angle_idx] != angles[-1]:
            while bez.motor_control.get_q(name) != angles[angle_idx]:
                bez.motor_control.set_single_motor(name, angles[angle_idx])

                # bez.motor_control.set_single_motor(name.replace("left", "right"), angles[angle_idx])
                bez.motor_control.set_motor()
                print(angles[angle_idx])
                # print(
                #     f" current Target angle: {angles[angle_idx]} {angle_idx} current joint angle: {bez.motor_control.get_q(name)}  current joint angle con: {bez.motor_control.get_control(name)}"
                # )
                sim.render(True)
                sim.step()

                if angles[angle_idx] < bez.motor_control.get_range(name)[0] or angles[angle_idx] > \
                        bez.motor_control.get_range(name)[1] or abs(bez.motor_control.get_qdot(name)) < 0.01:
                    break

            angle_idx += 1

            for i in range(10):
                sim.set_T_world_site("torso", T.matrix)
                sim.render(True)
                sim.step()

    # def test_foot_sensor(self): # TODO add to mcf model
    #     sim = SimWorld(scene_name="scene_bez2.xml")
    #     bez = Bez(sim)
    #     sim.step()
    #     T = np.eye(4)
    #     # T = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
    #     # T = Transformation(position=[0, 0, 0.070],
    #     #                    euler=[0, 1.57, 0])  # TODO maybe should be functions in world to streamline
    #     sim.set_T_world_site("left_foot", T)
    #
    #     sim.step()
    #     start = time.time()
    #
    #     while sim.t < 10:
    #         sim.render(True)
    #
    #         sim.step()
    #
    #         elapsed = time.time() - start
    #         frames = sim.frame
    #         print(bez.sensors.get_foot_pressure_sensors())
    #
    #          # TODO add assert
    #
    #         print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")


    def test_walk(self):
        sim = SimWorld(scene_name="scene_bez2.xml")
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        walk.wait(100)
        target_goal = [0.03, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 20:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            walk.walk(target_goal, display_metrics=False)
            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


        # walk.ready()
        # walk.wait(100)
        # target_goal = [0.08, 0, 0.0, 10, 500]
        # # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        # print("STARTING WALK")
        # for i in range(10000):
        #     # walk.walk(ball_pos, True)
        #     walk.walk(target_goal, display_metrics=False)
        #     # if not walk.enable_walking:
        #     #     print("WALK ENABLED")
        #     #     x = uniform(-1, 1) # TODO own unit test for yaw
        #     #     y = uniform(-1, 1)
        #     #     theta = uniform(-3.14, 3.14)
        #     #     print(x, y, theta)
        #     #     target_goal = Transformation(position=[x, y, 0], euler=[theta, 0, 0])
        #     #     walk.reset_walk()
        #     self.world.step()
        # walk.wait(10000)

    def test_ready(self):
        sim = SimWorld(scene_name="scene_bez2.xml")
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        walk.wait(100)
        target_goal = [0.08, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 20:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


