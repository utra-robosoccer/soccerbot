import os
import unittest
from os.path import expanduser
from random import uniform

import cv2
import numpy as np
import pybullet as pb
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_trajectories.trajectory_manager_sim import TrajectoryManagerSim

from soccer_common import Transformation

REAL_TIME = True


class TestPlaco(unittest.TestCase):
    def tearDown(self):
        self.world.close()
        del self.bez
        del self.world

    def test_bez1(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        tm = TrajectoryManagerSim(
            self.world, self.bez, os.path.join(os.path.dirname(__file__), "../../soccer_trajectories/trajectories/bez2/" + "rightkick_2" + ".csv")
        )
        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.ready()
        walk.wait(100)
        target_goal = [0.05, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        for i in range(10000):
            if i % 10 == 0:
                img = self.bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
                dimg, bbs_msg = detect.get_model_output(img)
                for box in bbs_msg.bounding_boxes:
                    if box.Class == "0":
                        detect.camera.pose.position = [0, 0, self.bez.sensors.get_pose(link=2).position[2]]
                        detect.camera.pose.orientation_euler = self.bez.sensors.get_pose(link=2).orientation_euler
                        # detect.camera.pose = self.bez.sensors.get_pose(link=2)
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        # print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                        kicked = False
                        ball_pos = self.bez.sensors.get_ball()
                        print(
                            f"floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position}  ball: {self.bez.sensors.get_ball().position}"
                        )
                        # pos = [box.xbase, box.ybase]
                        # # detect.camera.pose.position = self.bez.sensors.get_pose(link=2).position
                        # detect.camera.pose.position = [0, 0, self.bez.sensors.get_pose(link=2).position[2]]
                        # detect.camera.pose.orientation_euler = self.bez.sensors.get_pose(link=2).orientation_euler
                        # floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        # print(f"floor pos: {floor_coordinate_robot}  ball: {self.bez.sensors.get_ball().position}")
                        #
                        # ball_pos = Transformation(position=floor_coordinate_robot)
                        # temp = self.bez.sensors.get_pose().rotation_matrix @ self.bez.sensors.get_ball().position + self.bez.sensors.get_pose().position
                        # print(f"pos2: {temp} ball: {self.bez.sensors.get_ball_global().position}")
                if "DISPLAY" in os.environ:
                    cv2.imshow("CVT Color2", dimg)
                    cv2.waitKey(1)

            if 0 < np.linalg.norm(ball_pos.position[:2]) < 0.2 and not kicked:
                walk.ready()
                walk.wait(100)
                tm.send_trajectory()
                kicked = True
                # walk.kick_ready()
                # walk.kick()

                walk.reset_walk()
            else:
                walk.walk(ball_pos, True)
            # walk.walk(target_goal, display_metrics=False)
            # if not walk.enable_walking:
            #     print("WALK ENABLED")
            #     x = uniform(-1, 1) # TODO own unit test for yaw
            #     y = uniform(-1, 1)
            #     theta = uniform(-3.14, 3.14)
            #     print(x, y, theta)
            #     target_goal = Transformation(position=[x, y, 0], euler=[theta, 0, 0])
            #     walk.reset_walk()
            self.world.step()

    def test_bez1_walk(self):

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.ready()
        walk.wait(100)
        target_goal = [0.08, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        for i in range(10000):

            # walk.walk(ball_pos, True)
            walk.walk(target_goal, display_metrics=False)
            # if not walk.enable_walking:
            #     print("WALK ENABLED")
            #     x = uniform(-1, 1) # TODO own unit test for yaw
            #     y = uniform(-1, 1)
            #     theta = uniform(-3.14, 3.14)
            #     print(x, y, theta)
            #     target_goal = Transformation(position=[x, y, 0], euler=[theta, 0, 0])
            #     walk.reset_walk()
            self.world.step()
        walk.wait(10000)

    def test_bez1_kick(self):

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.kick_ready()
        walk.wait(200)
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        for i in range(10000):
            walk.kick()

            self.world.step()

    def test_bez_motor_range_single(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        # self.bez = Bez(robot_model="assembly", pose=Transformation())
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.ready()
        walk.wait(100)

        angles = np.linspace(-np.pi, np.pi)

        for j in angles:
            # if i % 10 == 0:
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))

            if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", img)
                cv2.waitKey(1)
            t = "shoulder_roll"
            self.bez.motor_control.configuration["right_" + t] = j
            self.bez.motor_control.configuration["left_" + t] = j
            # self.bez.motor_control.configuration[self.bez.motor_control.get_motor_array_index("head_yaw")] = j
            # x[self.bez.motor_control.get_motor_array_index("head_yaw")] = j
            # x[self.bez.motor_control.motor_names.index("right_"+t)] = j
            # x[self.bez.motor_control.motor_names.index("left_"+t)] = j
            self.bez.motor_control.set_motor()

            self.world.wait_motor()

        for i in range(10000):
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))

            if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", img)
                cv2.waitKey(1)
            self.world.step()

    def test_bez1_start_stop(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.ready()
        walk.wait(100)
        target_goal = [1, 0, 0, 10, 2]
        print("STARTING WALK")

        for i in range(10000):
            walk.walk(target_goal, display_metrics=False)
            if i % 1000 == 0:
                walk.reset_walk()

            self.world.step()

    def test_bez1_ready(self):
        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        # TODO should bez be init in walk_engine
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        walk.ready()
        walk.world.step()
        walk.wait(100)
