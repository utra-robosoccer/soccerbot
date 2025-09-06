import unittest

import numpy as np
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.mujoco.simulator import Simulator

from soccer_common import Transformation

REAL_TIME = True


class TestMujoco(unittest.TestCase):
    def tearDown(self):
        # MuJoCo doesn't need explicit closing like PyBullet
        del self.bez
        del self.world

    def test_imu(self):
        self.world = Simulator(scene_name="scene_bez2.xml")
        pose = Transformation()
        # pose = Transformation(position=[0, 0, 0.070], euler=[0, -1.57, 0])
        # pose = Transformation(position=[0, 0, 0.070], euler=[0, 1.57, 0])
        self.bez = Bez(robot_model="assembly", pose=pose, simulator=self.world)
        # Wait equivalent in MuJoCo
        # for _ in range(100):
        #     self.world.render(True)
        #     self.world.step()
        self.world.step()
        self.world.set_T_world_site("left_foot", np.eye(4))

        self.world.step()
        for i in range(100000):
            self.world.render(True)
            [_, pitch, roll] = self.bez.sensors.get_imu()
            print(pitch, roll)
            self.world.step()
        # TODO add more

    def test_foot_sensor(self):
        self.world = Simulator(scene_name="scene_bez2.xml")
        self.bez = Bez(robot_model="bez2", pose=Transformation(), simulator=self.world)
        # Wait equivalent in MuJoCo
        for _ in range(100):
            self.world.step()
        print(self.bez.sensors.get_foot_pressure_sensors())
        # TODO add more and fix the link location

    def test_bez_motor_range(self):
        self.world = Simulator(scene_name="scene_assembly.xml")
        self.bez = Bez(robot_model="assembly", fixed_base=True, pose=Transformation(), simulator=self.world)
        # Wait equivalent in MuJoCo
        for _ in range(50):
            self.world.step()
        angles = np.linspace(-np.pi, np.pi)
        for i in range(self.bez.motor_control.numb_of_motors):
            for j in angles:
                x = [0.0] * self.bez.motor_control.numb_of_motors
                x[i] = j
                self.bez.motor_control.set_motor_positions(x)
                self.world.step()

        # Additional wait
        for _ in range(100):
            self.world.step()

    def test_bez_motor_range_single(self):
        self.world = Simulator(scene_name="scene_assembly.xml")
        # self.bez = Bez(robot_model="bez1", fixed_base=True, pose=Transformation(), simulator=self.world)
        self.bez = Bez(robot_model="assembly", fixed_base=True, pose=Transformation(), simulator=self.world)
        # Wait equivalent in MuJoCo
        for _ in range(50):
            self.world.step()
        angles = np.linspace(-np.pi, np.pi)

        for j in angles:
            # x = [0.0] * self.bez.motor_control.numb_of_motors
            t = "hip_yaw"
            # x[self.bez.motor_control.motor_names.index("head_pitch")] = j
            self.bez.motor_control.configuration["right_" + t] = j
            self.bez.motor_control.configuration["left_" + t] = j
            # x[self.bez.motor_control.motor_names.index("right_"+t)] = j
            # x[self.bez.motor_control.motor_names.index("left_"+t)] = j

            self.bez.motor_control.set_motor()
            self.world.step()

        # Additional wait
        for _ in range(100):
            self.world.step()

    # TODO fix or wait for mujoco?
    # def test_stand_plane(self):
    #     self.world = Simulator(scene_name="scene_bez2.xml")
    #     self.bez = Bez(pose=Transformation(), simulator=self.world)
    #     tf = Navigator(self.world, self.bez)
    #     # Wait equivalent in MuJoCo
    #     for _ in range(50):
    #         self.world.step()
    #     tf.bez.ready()
    #     # Wait equivalent in MuJoCo
    #     for _ in range(50):
    #         self.world.step()
    #     t = 0
    #     while t < 1000:
    #         [_, pitch, roll] = tf.bez.sensors.get_imu()
    #         # if tf.fallen(pitch):
    #         #     # Apply external force in MuJoCo
    #         #     # self.world.apply_force("torso", [0, 5, 0])
    #         #     pass
    #         # p.model.set_pose()
    #         # TODO fix link alignment
    #         # Apply external force in MuJoCo
    #         # self.world.apply_force("torso", [3, 0, 0])
    #         tf.stabilize_stand(pitch, roll)
    #         self.world.step()
    #         t += 0.01
    #     # Additional wait
    #     for _ in range(100):
    #         self.world.step()
