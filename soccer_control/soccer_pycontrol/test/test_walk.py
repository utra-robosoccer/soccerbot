import time
import unittest
from os.path import expanduser
from random import uniform
from test.keyboard_gamepad import KeyboardGamepad

import cv2
import mujoco
import numpy as np
import onnxruntime as rt
from etils import epath
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.model.sim_world import SimWorld
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_pycontrol.walk_engine.rl_walk import RLWalk
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

from soccer_common import Transformation

REAL_TIME = True


class TestMuJoCo(unittest.TestCase):
    def test_walk_policy(self):
        sim = SimWorld(keyframe="stand")
        bez = Bez(sim)
        start = time.time()

        ctrl_dt = 0.02
        sim_dt = 0.002
        n_substeps = int(round(ctrl_dt / sim_dt))

        policy = RLWalk(
            policy_name="bez222_policy.onnx",
            bez=bez,
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
        )
        while sim.t < 3000:

            policy.cmd = [1, 0, 0]

            policy.get_control()

            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame

            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_walk_placo(self):  # TODO get placo workign as a base line
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        walk.world.step()
        sim.wait(200)
        # walk.wait(100)
        target_goal = [0.1, 0.0, 0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        start = time.time()
        print("STARTING WALK")
        s = time.time()
        while sim.t < 20:

            if sim.frame % int(walk.foot_step_planner.DT / sim.dt) == 0:  # TODO investigate interp
                walk.walk(target_goal, display_metrics=False)
                print("here: " + str(1 / (time.time() - s)))
                s = time.time()

            sim.render(True)
            sim.step()
            elapsed = time.time() - start
            frames = sim.frame
            print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_walk_rand(self):  # TODO needs tuning
        sim = SimWorld()
        bez = Bez(sim)
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        sim.wait(200)

        target_goal = Transformation(position=[0.07, -0.00, 0], euler=[0, 0, 0])
        ctrl_dt = 0.02
        sim_dt = 0.002
        n_substeps = int(round(ctrl_dt / sim_dt))

        policy = RLWalk(
            policy_name="bez222_policy.onnx",
            bez=bez,
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
        )
        start = time.time()
        x, y, theta = 0, 0, 0
        print("STARTING WALK")
        while sim.t < 1000:
            sim.render(True)

            sim.step()

            elapsed = time.time() - start
            frames = sim.frame
            # print(bez.sensors.get_pose().position)
            if not walk.walker.enable_walking:
                print("WALK ENABLED")
                x = uniform(-1, 1)  # TODO own unit test for yaw
                y = uniform(-1, 1)
                theta = uniform(-3.14, 3.14)
                print("here: ", x, y, theta)
                target_goal = Transformation(position=[x, y, 0], euler=[theta, 0, 0])
                walk.walker.reset_walk()
            if sim.frame % int(walk.foot_step_planner.DT / sim.dt) == 0:  # TODO investigate interp
                walk.walk(target_goal, display_metrics=False)
                # print(list(policy.cmd))
                # print("here: ", x, y, theta)
                # print(f"ex: {x - bez.sensors.get_pose().position[0]} ey: {y - bez.sensors.get_pose().position[1]} etheta: {theta - bez.sensors.get_pose().orientation_euler[0]}")

            # walk.walk(target_goal, display_metrics=False)
            policy.cmd[0] = walk.dx * 12.5
            policy.cmd[1] = walk.dy * 12.5
            policy.cmd[2] = walk.dtheta * 3

            policy.get_control()

            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_full(self):  # TODO should be in stratgy
        src_path = expanduser("~") + "/ros2_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        sim = SimWorld()
        bez = Bez(sim)
        tm = TrajectoryManagerSim(sim, bez, "bez2", "getupfront")  # TODO nav and traj should not require world
        walk = Navigator(sim, bez, imu_feedback_enabled=True)
        walk.ready()
        sim.wait(200)
        sim.step()
        ctrl_dt = 0.02
        n_substeps = int(round(ctrl_dt / sim.dt))

        policy = RLWalk(
            policy_name="bez222_policy.onnx",
            bez=bez,
            ctrl_dt=ctrl_dt,
            n_substeps=n_substeps,
        )
        start = time.time()
        x, y, theta = 0, 0, 0
        start = time.time()
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        ball_pixel = [0, 0]
        while sim.t < 300:
            if sim.frame % int((1 / 30.0) / sim.dt) == 0:
                img = bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                dimg, bbs_msg = detect.get_model_output(img)
                for box in bbs_msg.bounding_boxes:
                    if box.class_id == "0":
                        detect.camera.pose.position = [0, 0, bez.sensors.get_cam_pose().position[2]]
                        detect.camera.pose.orientation_euler = bez.sensors.get_cam_pose().orientation_euler
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        ball_pixel = [(box.xmin + box.xmax) / 2.0, (box.ymin + box.ymax) / 2.0]
                        kicked = False
                        # ball_pos = bez.sensors.get_ball_local_frame()
                        pos = [box.xbase, box.ybase]
                        floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        string = (
                            f"z: {bez.sensors.get_cam_pose().position[2]}  eul: {bez.sensors.get_cam_pose().orientation_euler}"
                            + f" floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position} ball: {bez.sensors.get_ball_local_frame().position}"
                            + f" floor pos2: {floor_coordinate_robot}  ball: {bez.sensors.get_ball_local_frame().position}"
                        )

                        print(
                            string,  # end='\r',
                            flush=True,
                        )

                        ball_pos = Transformation(position=floor_coordinate_robot)

                # if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)
            if 0 < np.linalg.norm(ball_pos.position[:2]) < 0.0 and not kicked:
                walk.ready()
                walk.wait(100)
                tm.send_trajectory("rightkick")
                kicked = True

                walk.walker.reset_walk()
            else:

                if sim.frame % int(walk.foot_step_planner.DT / sim.dt) == 0:  # TODO investigate interp
                    walk.walk(ball_pos, ball_pixel, True)
                    # print("here: " + str(1 / (time.time() - s)))
                    # s = time.time()
                    # print("ba;ll: " + str(np.linalg.norm(ball_pos.position[:2])))
                policy.cmd[0] = walk.dx * 12.5
                policy.cmd[1] = walk.dy * 12.5
                policy.cmd[2] = walk.dtheta * 3
                policy.get_control()

                # walk.walk(ball_pos, ball_pixel, True)
                # print( "ba;ll: "+ str(np.linalg.norm(ball_pos.position[:2])) )
            elapsed = time.time() - start
            frames = sim.frame
            sim.render(True)
            sim.step()
            # print(bez.fallen(pitch))  # TODO add assert

            # print(f"Elapsed: {elapsed:.2f}, Frames: {frames}, FPS: {frames / elapsed:.2f}")

        sim.close_viewer()

    def test_bez1_start_stop(self):  # TODO dont know if we need this
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
