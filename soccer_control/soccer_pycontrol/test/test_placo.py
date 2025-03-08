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
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation(), fixed_base=False)
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        walk.ready()
        self.world.wait(100)
        target_goal = [0.05, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        ball_pixel = [0, 0]
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
                        ball_pixel = [(box.xmin + box.xmax) / 2.0, (box.ymin + box.ymax) / 2.0]
                        # print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                        kicked = False
                        ball_pos = self.bez.sensors.get_ball()
                        print(
                            f"floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position}   ball: {self.bez.sensors.get_ball().position}",
                            flush=True,
                        )
                        pos = [box.xbase, box.ybase]
                        # detect.camera.pose.position = self.bez.sensors.get_pose(link=2).position
                        detect.camera.pose.position = [0, 0, self.bez.sensors.get_pose(link=2).position[2]]
                        detect.camera.pose.orientation_euler = self.bez.sensors.get_pose(link=2).orientation_euler
                        floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                        print(f"floor pos2: {floor_coordinate_robot}  ball: {self.bez.sensors.get_ball().position}")
                        # print(f"z: {self.bez.sensors.get_pose(link=2).position}  eul: {self.bez.sensors.get_pose(link=2).orientation_euler}  motor: {self.bez.motor_control.configuration["head_yaw"]}")
                        # ball_pos = Transformation(position=floor_coordinate_robot)
                        # temp = self.bez.sensors.get_pose().rotation_matrix @ self.bez.sensors.get_ball().position + self.bez.sensors.get_pose().position
                        # print(f"pos2: {temp} ball: {self.bez.sensors.get_ball_global().position}")
                        # temp1 = detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position
                        # temp2 = self.bez.sensors.get_ball().position
                font = cv2.FONT_HERSHEY_DUPLEX
                color = (255, 255, 255)  # red
                fontsize = 255
                text = "test"
                position = (10, 10)

                # cv2.putText(dimg, text, position, font, fontsize, color=color)
                if "DISPLAY" in os.environ:
                    cv2.imshow("CVT Color2", dimg)
                    cv2.waitKey(1)

            if 0 < np.linalg.norm(ball_pos.position[:2]) < 0.2 and not kicked:
                walk.ready()
                walk.wait(100)
                tm.send_trajectory("rightkick")
                kicked = True
                # walk.kick_ready()
                # walk.kick()

                walk.reset_walk()
            else:

                pass
                walk.walk(ball_pos, ball_pixel, True)
            # print(f"Height rotation: {self.bez.sensors.get_height().orientation_euler}", flush=True)
            # print(f"Height position: {self.bez.sensors.get_height().position}", flush=True)

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

    def test_camera(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"
        detect = ObjectDetectionNode(model_path)

        cap = cv2.VideoCapture(4)
        if not cap.isOpened():
            print("Cannot open camera")
            exit()

        # self.world = PybulletWorld(
        #     camera_yaw=90,
        #     real_time=REAL_TIME,
        #     rate=200,
        # )
        # self.bez = Bez(robot_model="assembly", pose=Transformation())
        # tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        # self.bez = Bez(robot_model="bez1", pose=Transformation())
        # walk = Navigator(self.world, self.bez, imu_feedback_enabled=False, ball=True)
        # walk.ready()
        # walk.wait(100)
        target_goal = [0.05, 0, 0.0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        kicked = False
        while True:
            # img = self.bez.sensors.get_camera_image()
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break
            img = cv2.resize(frame, dsize=(640, 480))
            # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            # dimg, bbs_msg = detect.get_model_output_v8(img)
            dimg, bbs_msg = detect.get_model_output(img)
            for box in bbs_msg.bounding_boxes:
                if box.Class == "0":
                    detect.camera.pose.position = [0, 0, 0.6]
                    # detect.camera.pose.orientation_euler = self.bez.sensors.get_pose(link=2).orientation_euler
                    detect.camera.pose.orientation_euler = [-0.029378, -0.11132, 0.063983]
                    # detect.camera.pose = self.bez.sensors.get_pose(link=2)
                    boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                    # print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                    # kicked = False
                    # ball_pos = self.bez.sensors.get_ball()
                    print(f"floor pos: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position}", flush=True)
                    # print(detect.camera.pose.orientation_euler)
                    print()

            if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

        # self.world.step()

    def test_bez1_walk(self):

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

    def test_bez1_auto(self):

        self.world = PybulletWorld(
            camera_yaw=90,
            real_time=REAL_TIME,
            rate=200,
        )
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        tm = TrajectoryManagerSim(self.world, self.bez, "bez2_sim", "getupfront")

        for i in range(100000):
            y, p, r = self.bez.sensors.get_imu()
            print(y, "  ", p, "  ", r)
            if p > 1.25:
                print("getupfront")
                tm.send_trajectory("getupfront")
            elif p < -1.25:
                print("getupback: ")
                tm.send_trajectory("getupback")
            elif r < -1.54 and -0.5 < p < -0.4:
                tm.send_trajectory("getupsideleft")
            elif r > 1.54 and -0.5 < p < -0.4:
                tm.send_trajectory("getupsideright")
            self.world.step()

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
        self.bez = Bez(robot_model="assembly", pose=Transformation())
        walk = Navigator(self.world, self.bez)
        walk.ready()
        walk.world.step()
        self.world.wait(100)
