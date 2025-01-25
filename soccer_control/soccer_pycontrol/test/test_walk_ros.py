import os
import time
import unittest
from os.path import expanduser

import cv2
import rospy
from geometry_msgs.msg import PoseStamped

from soccer_object_detection.object_detect_node import ObjectDetectionNode
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.walk_engine_ros.navigator_ros import NavigatorRos

from soccer_common import Transformation

os.environ["ROS_NAMESPACE"] = "/robot1"


class TestPybullet(unittest.TestCase):
    @unittest.skipIf("DISPLAY" not in os.environ, "only local")
    def test_walk_ros_local(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        # os.system(
        #     f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        # )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = NavigatorRos(bez, imu_feedback_enabled= True, ball2=True)
        # walker.wait(50)
        walker.ready()
        # bez.motor_control.set_single_motor("head_pitch", 0.7)
        # bez.motor_control.set_motor()
        walker.wait(50)
        # walker.goal_callback(PoseStamped())
        # walker.walk(d_x=0.04, t_goal=10)
        # target_goal = Transformation(position=[1, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
        # target_goal = [0.03     , 0.0, 0, 10, 500]
        target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
        walker.run(target_goal)

        walker.wait(100)

    def test_ready_ros_local(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        # os.system(
        #     f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        # )

        rospy.init_node("soccer_control")

        bez = BezROS()
        walker = NavigatorRos(bez, imu_feedback_enabled= True, ball2=True)
        # walker.wait(50)
        walker.ready()
        walker.wait(50)
        # walker.goal_callback(PoseStamped())
        # walker.walk(d_x=0.04, t_goal=10)
        # target_goal = Transformation(position=[1, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
        # target_goal = [0.03, 0.0, 0, 10, 500]
        # target_goal = Transformation(position=[0, 0, 0], euler=[0, 0, 0])
        # # walker.walk(target_goal)
        # walker.run(target_goal)

        walker.wait(100)
    def test_camera2(self):
        robot_ns = os.environ["ROS_NAMESPACE"]


        rospy.init_node("soccer_control")
        bez = BezROS()



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
        bez.motor_control.set_single_motor("head_yaw", 0)
        rospy.sleep(0.5)
        bez.motor_control.set_motor()
        rospy.sleep(0.5)
        rospy.sleep(1)
        print(f"Height rotation: {bez.sensors.get_height().orientation_euler}")
        print(f"Height position: {bez.sensors.get_height().position}")
        bez.motor_control.set_single_motor("head_yaw", 1)
        rospy.sleep(0.5)
        bez.motor_control.set_motor()
        rospy.sleep(0.5)
        print(f"Height rotation2: {bez.sensors.get_height().orientation_euler}")
        print(f"Height position2: {bez.sensors.get_height().position}")

    def test_camera(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        # os.system(
        #     f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        # )

        rospy.init_node("soccer_control")
        bez = BezROS()

        walker = NavigatorRos(bez, imu_feedback_enabled=True, ball2=True)
        src_path = expanduser("~") + "/catkin_ws/src/soccerbot/soccer_perception/"
        model_path = src_path + "soccer_object_detection/models/yolov8s_detect_best.pt"
        model_path = src_path + "soccer_object_detection/models/half_5.pt"
        detect = ObjectDetectionNode(model_path)

        # target_goal = Transformation(position=[1, 0, 0], euler=[0, 0, 0])
        # walker.walk(target_goal)
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
        while  not rospy.is_shutdown():
            # img = self.bez.sensors.get_camera_image()
            # bez.motor_control.set_single_motor("head_pitch", 0.7)
            walker.ready()
            bez.motor_control.set_motor()
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
                    detect.camera.pose.position = [0, 0,bez.sensors.get_height().position[2]]
                    # detect.camera.pose.orientation_euler = self.bez.sensors.get_pose(link=2).orientation_euler
                    detect.camera.pose.orientation_euler = [-0.029378, -0.11132, 0.063983]
                    detect.camera.pose.orientation_euler = bez.sensors.get_height().orientation_euler
                    # detect.camera.pose = self.bez.sensors.get_pose(link=2)
                    boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                    # print(detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position)
                    # kicked = False
                    ball_pos = detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes)
                    print(f"floor pos1: {detect.camera.calculate_ball_from_bounding_boxes(boundingBoxes).position}", flush=True)

                    pos = [box.xbase, box.ybase]
                    floor_coordinate_robot = detect.camera.find_floor_coordinate(pos)

                    print(f"floor pos2: {floor_coordinate_robot}  ", flush=True)

                    # print(detect.camera.pose.orientation_euler)
                    print(f"Height rotation: {bez.sensors.get_height().orientation_euler}", flush=True)
                    print(f"Height position: {bez.sensors.get_height().position}", flush=True)

            if "DISPLAY" in os.environ:
                cv2.imshow("CVT Color2", dimg)
                cv2.waitKey(1)

        # self.world.step()

    @unittest.skipIf("DISPLAY" not in os.environ, "only local")
    def test_walk_ros_webots(self):
        robot_ns = os.environ["ROS_NAMESPACE"]
        os.system(
            f"/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill {robot_ns}/soccer_strategy {robot_ns}/soccer_pycontrol {robot_ns}/soccer_trajectories'"
        )
        rospy.init_node("soccer_control")
        ns = "/robot1/"
        bez = BezROS(ns=ns)
        tmp = rospy.Publisher(ns + "reset_robot", PoseStamped)
        # tmp = rospy.Publisher(ns + "reset_robot", PoseStamped)
        walker = NavigatorRos(bez, ball2=True)
        walker.bez.motor_control.configuration.reset()
        walker.bez.motor_control.set_motor()
        walker.wait(50)
        tmp.publish(PoseStamped())
        walker.wait(50)
        # walker.ready()
        # bez.motor_control.set_motor()
        # walker.wait(50)
        # walker.goal_callback(PoseStamped())
        start = time.time()
        # target_goal = Transformation(position=[1, 0, 0], euler=[0,0,0])
        # walker.walk(target_goal)
        walker.run(bez.sensors.get_ball())
        # target_goal = [0.0, 0, 0, 10, 500]
        # walker.walk(target_goal)
        print("cm/s: ", 100 / (time.time() - start))
        # walker.bez.sensors.get_ball(rospy.Time.now())
        walker.wait(1000000)
