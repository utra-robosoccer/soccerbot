import os
import unittest
from os.path import expanduser
from random import uniform

import cv2
from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.bez import Bez
from soccer_pycontrol.pybullet_usage.pybullet_world import PybulletWorld
from soccer_pycontrol.walk_engine.navigator import Navigator

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
        # self.bez = Bez(robot_model="assembly", pose=Transformation())
        self.bez = Bez(robot_model="bez1", pose=Transformation())
        walk = Navigator(self.world, self.bez, imu_feedback_enabled=False)
        walk.ready()
        walk.wait(100)
        target_goal = [1, 0, 0, 10, 2]
        target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        print("STARTING WALK")
        ball_pos = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        for i in range(10000):
            if i % 10 == 0:
                img = self.bez.sensors.get_camera_image()
                img = cv2.resize(img, dsize=(640, 480))
                # detect.camera.pose.orientation_euler = [0, np.pi / 8, 0]
                dimg, bbs_msg = detect.get_model_output(img)
                for box in bbs_msg.bounding_boxes:
                    if box.Class == "0":
                        detect.camera.pose = self.bez.sensors.get_pose(link=2)
                        boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                        print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                        ball_pos = detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes)
                if "DISPLAY" in os.environ:
                    cv2.imshow("CVT Color2", dimg)
                    cv2.waitKey(1)

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
