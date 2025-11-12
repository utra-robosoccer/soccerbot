import time
import unittest
from random import uniform

import numpy as np
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.sim_world import SimWorld

from soccer_common import Transformation
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

REAL_TIME = True


class TestMuJoCo(unittest.TestCase):
    def test_imu(self):
        sim = SimWorld(keyframe="fallen_side_right")
        bez = Bez(sim)
        start = time.time()

        while sim.t < 3:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            [_, pitch, roll] = bez.sensors.get_imu()
            print(bez.fallen(pitch, roll))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_ball_pose(self):
        # sim = SimWorld()
        sim = SimWorld(pose=Transformation(position=[1, 1,0.3975], euler=[0, 0, 0]))
        bez = Bez(sim)
        sim.set_T_world_ball(Transformation(position=[2, -1, 0.14], euler=[0, 0, 0]))
        sim.step()
        start = time.time()

        while sim.t < 20:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            print("pose: ", bez.sensors.get_pose().position)
            print("local: ", bez.sensors.get_ball_local_frame().position)  # TODO add assert
            print("Global: ", bez.sensors.get_ball_world_frame().position)
            print("2Global: ", bez.sensors.get_head_height())
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_bez_motor_range_single(self):
        sim = SimWorld(keyframe="float")
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
        sim = SimWorld(keyframe="float")
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
                sim.set_T_world_site("torso", T.matrix) # todo maybe could make this more of a built in fucntion
                sim.render(True)
                sim.step()

    # def test_foot_sensor(self): # TODO add to mcf model
    #     sim = SimWorld()
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
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 20:
            sim.render(True)
            sim.step()
            elapsed = time.time() - start
            frames = sim.frame
            walk.walk(target_goal, display_metrics=False)

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


    def test_walk_rand(self): # TODO needs tuning
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0, 0.0, 10, 500]
        target_goal = Transformation(position=[0.08, -0.06, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 10:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame

            if not walk.walker.enable_walking:
                print("WALK ENABLED")
                x = uniform(-1, 1) # TODO own unit test for yaw
                y = uniform(-1, 1)
                theta = uniform(-3.14, 3.14)
                print(x, y, theta)
                target_goal = Transformation(keyframe=[x, y, 0], euler=[theta, 0, 0])
                walk.walker.reset_walk()
            walk.walk(target_goal, display_metrics=False)
            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()



    def test_ready(self):
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()

        start = time.time()
        print("STARTING WALK")
        while sim.t < 3:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()


    def test_bez1_start_stop(self):
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=False)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0, 0.0, 10, 2]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        while sim.t < 10:
            sim.render(True)
            sim.step()
            elapsed = time.time() - start
            frames = sim.frame
            walk.walk(target_goal)
            if sim.t % 3 == 0:
                walk.walker.reset_walk()
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()



    def test_bez1_auto(self):
        sim = SimWorld(keyframe="fallen_side_left")
        bez = Bez(sim)
        tm = TrajectoryManagerSim(sim, bez, "bez2", "getupfront")
        start = time.time()

        while sim.t < 50:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            y, p, r = bez.sensors.get_imu()
            print(y, "  ", p, "  ", r)
            traj = bez.fallen(p,r)
            tm.send_trajectory(traj)


            # print(bez.fallen(pitch))  # TODO add assert

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()




# TODO test stand plane - apply force
