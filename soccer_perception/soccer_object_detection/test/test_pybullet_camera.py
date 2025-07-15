import os
import unittest
from os.path import expanduser

import cv2
import numpy as np
import pybullet as pb
import pytest
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld

from soccer_common import Transformation

REAL_TIME = True
PLOT = True


class TestPybullet(unittest.TestCase):
    def tearDown(self):
        self.world.close()
        del self.bez
        del self.world

    def test_cam(self):

        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        pose = Transformation()
        self.bez = Bez(robot_model="assembly", pose=pose)
        for i in range(100):
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))

            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("CVT Color2", img)
                cv2.waitKey(1)

            self.world.step()

    def test_detection(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        pose = Transformation()
        self.bez = Bez(robot_model="assembly", pose=pose)

        self.world.wait(100)
        for i in range(100):
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))
            detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            dimg, bbs = detect.get_model_output(img)
            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

            self.world.step()

    def test_ball_localization(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)

        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        pose = Transformation()
        self.bez = Bez(robot_model="assembly", pose=pose)

        for i in range(1000):
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))
            # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            dimg, bbs_msg = detect.get_model_output(img)
            for box in bbs_msg.bounding_boxes:
                if box.data == "0":
                    detect.camera.pose = self.bez.sensors.get_pose(link=2)
                    boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                    print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

            self.world.step()

    def test_ball_localization2(self):
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"

        detect = ObjectDetectionNode(model_path)
        self.world = PybulletWorld(camera_yaw=45, real_time=REAL_TIME, rate=100)
        pose = Transformation()
        self.bez = Bez(robot_model="assembly", pose=pose)

        for i in range(1000):
            img = self.bez.sensors.get_camera_image()
            img = cv2.resize(img, dsize=(640, 480))
            # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
            dimg, bbs_msg = detect.get_model_output(img)
            for box in bbs_msg.bounding_boxes:
                if box.Class == "0":
                    pos = [box.xbase, box.ybase]
                    floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)
                    world_to_obstacle = Transformation(position=floor_coordinate_robot)
                    detect.camera.pose = self.bez.sensors.get_pose(link=1)
                    # detect.camera.pose.orientation_euler = [0, 0, 0]
                    camera_to_obstacle = np.linalg.inv(detect.camera.pose) @ world_to_obstacle
                    print(camera_to_obstacle.position, "   ", self.bez.sensors.get_pose(link=1).position)
            if "DISPLAY" in os.environ and PLOT:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

            self.world.step()
