import time
import unittest
from os.path import expanduser

import cv2
import numpy as np
import pytest
from soccer_pycontrol.mujoco.bez import Bez
from soccer_pycontrol.mujoco.simulator import SimWorld

from soccer_common import Transformation

REAL_TIME = True


class TestPybullet(unittest.TestCase):
    # def tearDown(self):
    #     self.world.close()
    #     del self.bez
    #     del self.world

    def test_imu(self):
        sim = SimWorld(scene_name="scene_bez2.xml")
        bez = Bez(sim)
        sim.step()
        # T = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
        T = Transformation(position=[0, 0, 0.070], euler=[0, 1.57, 0])  # TODO maybe should be functions in world to streamline
        sim.set_T_world_site("left_foot", T.matrix)

        sim.step()
        start = time.time()

        while sim.t < 2:  # TODO add a time limit
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            [_, pitch, roll] = bez.sensors.get_imu()

            print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

    def test_bez_motor_range_single(self):
        sim = SimWorld(scene_name="scene_bez2.xml")
        bez = Bez(sim)
        sim.step()
        # T = np.eye(4)
        # sim.set_T_world_site("left_foot", T)
        T = Transformation(position=[0, 0, 0.6], euler=[0, 0, 0])  # TODO maybe should be functions in world to streamline
        sim.set_T_world_site("torso", T.matrix)
        sim.step()
        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()
        name = "left_shoulder_pitch"
        # for name in bez.motor_control.dof_names:
        angle_idx = 0
        # bez.motor_control.clear_motor_target()
        while angles[angle_idx] != angles[-1]:
            while bez.motor_control.get_control(name) != angles[angle_idx]:
                bez.motor_control.set_control(name, angles[angle_idx])
                bez.motor_control.set_motor()
                print(
                    f" current Target angle: {angles[angle_idx]} {angle_idx} current joint angle: {bez.motor_control.get_q(name)}  current joint angle con: {bez.motor_control.get_control(name)}"
                )
                sim.step()
                sim.set_T_world_site("torso", T.matrix)
                sim.step()
                sim.render(True)
                sim.step()
            angle_idx += 1
            # sim.set_T_world_site("torso", T.matrix)
            # sim.render(True)
            # sim.step()
            for i in range(100):
                sim.step()
                sim.set_T_world_site("torso", T.matrix)
                sim.step()
                sim.render(True)
                sim.step()
            # sim.wait(100)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

    def test_bez_motor_range(self):
        sim = SimWorld(scene_name="scene_bez2.xml")
        bez = Bez(sim)
        # sim.set_gravity([0,0,0])
        sim.step()
        # T = np.eye(4)
        # sim.set_T_world_site("left_foot", T)
        T = Transformation(position=[0, 0, 0.6], euler=[0, 0, 0])  # TODO maybe should be functions in world to streamline
        sim.set_T_world_site("torso", T.matrix)

        sim.step()
        start = time.time()
        angles = np.linspace(-np.pi, np.pi)

        sim.render(True)

        sim.step()

        for name in bez.motor_control.dof_names:
            angle_idx = 0
            bez.motor_control.clear_motor_target()
            bez.motor_control.reset_motors(np.zeros_like(bez.motor_control.configuration))
            sim.render(True)
            sim.step()
            name = "left_shoulder_pitch"
            while angles[angle_idx] != angles[-1]:
                while bez.motor_control.get_control(name) != angles[angle_idx]:
                    bez.motor_control.set_control(name, angles[angle_idx])
                    bez.motor_control.set_motor()
                    print(
                        f" current Target angle: {angles[angle_idx]} {angle_idx} current joint angle: {bez.motor_control.get_q(name)}  current joint angle con: {bez.motor_control.get_control(name)}"
                    )
                    sim.set_T_world_site("torso", T.matrix)
                    sim.render(True)
                    sim.step()
                angle_idx += 1
                for i in range(100):
                    sim.set_T_world_site("torso", T.matrix)
                    sim.render(True)
                    sim.step()

                # sim.wait(100)

        sim.wait(1000)
        elapsed = time.time() - start
        frames = sim.frame

        print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

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
    #     while sim.t < 10:  # TODO add a time limit
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
