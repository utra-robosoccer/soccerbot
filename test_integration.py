import os
import signal
import subprocess
import sys
from unittest import TestCase

import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as R
from timeout_decorator import timeout_decorator

from soccer_common.camera import Camera
from soccer_msgs.msg import BoundingBoxes, RobotState
from soccer_strategy.src.soccer_strategy.team import Team

RUN_LOCALLY = "pycharm" in sys.argv[0]


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    MAGENTA = "\u001b[35m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def printlog(s: str):
    print(bcolors.MAGENTA + s + bcolors.ENDC)


class IntegrationTest(TestCase):
    START_PLAY = "false"
    ROBOT_MODEL = "bez1"

    def reset_simulation(self):

        if RUN_LOCALLY:
            if hasattr(self, "simulation_process"):
                self.simulation_process.send_signal(signal.SIGINT)
                self.simulation_process.wait(10)

            subprocess.call(["/bin/bash", "-c", "killall python3 || echo 'No Python Executables running'"])
            subprocess.call(["/bin/bash", "-c", "killall /usr/bin/java || echo 'No Java Executables running'"])
            subprocess.call(["/bin/bash", "-c", "kill -9 $(pgrep webots) || echo 'No Webots running'"])
            subprocess.call(["/bin/bash", "-c", "source ~/catkin_ws/devel/setup.bash && rosnode kill -a || echo 'No Nodes running"])

    def start_simulation(self):
        if RUN_LOCALLY:
            self.reset_simulation()
            self.simulation_process = subprocess.Popen(
                [
                    "/bin/bash",
                    "-c",
                    f"export START_PLAY={self.START_PLAY} && export ROBOT_MODEL={self.ROBOT_MODEL} && source ~/catkin_ws/devel/setup.bash && roslaunch soccerbot soccerbot_multi.launch",
                ]
            )
        else:
            os.system("bash $HOME/catkin_ws/src/soccerbot/soccerbot/scripts/start_competition.sh robot$ROBOCUP_ROBOT_ID &")

        rospy.init_node("integration_test")
        rospy.wait_for_message("/clock", Clock, 40)
        self.camera = Camera("robot1")
        self.tf_listener = tf.TransformListener()
        self.bounding_boxes_detector = rospy.Subscriber("/robot1/object_bounding_boxes", BoundingBoxes, self.bounding_boxes_callback)
        self.bounding_boxes = None

        self.clock_sub = rospy.Subscriber("/clock", Clock, self.clock_callback)
        self.clock_pub = rospy.Publisher("/clock_test", Clock, queue_size=1)

    def clock_callback(self, c: Clock):
        c.clock.secs += 1
        self.clock_pub.publish(c)

    def bounding_boxes_callback(self, b: BoundingBoxes):
        self.bounding_boxes = b

    def set_robot_pose(self, x, y, theta):
        resetPublisher = rospy.Publisher("/robot1/reset_robot", PoseStamped, queue_size=1, latch=True)
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "world"
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0

        r = R.from_euler("ZYX", [theta, 0, 0], degrees=False)
        q = r.as_quat()

        p.pose.orientation.x = q[0]
        p.pose.orientation.y = q[1]
        p.pose.orientation.z = q[2]
        p.pose.orientation.w = q[3]

        resetPublisher.publish(p)

    def set_ball_pose(self, x, y):
        ballPublisher = rospy.Publisher("/reset_ball", PoseStamped, queue_size=1, latch=True)
        p = PoseStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = "world"
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0

        ballPublisher.publish(p)

    def get_ball_pose(self, gt=False):
        try:
            if gt:
                frame = "robot1/ball_gt"
            else:
                frame = "robot1/ball"
            last_observed_time_stamp = self.tf_listener.getLatestCommonTime("world", frame)
            ball_pose = self.tf_listener.lookupTransform("world", frame, last_observed_time_stamp)
            return np.array([ball_pose[0][0], ball_pose[0][1], ball_pose[0][2]])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate ball in TF tree")
            return None

    def setUp(self) -> None:
        self.start_simulation()
        super().setUp()

    def tearDown(self) -> None:
        super(IntegrationTest, self).tearDown()
        self.reset_simulation()

    def __del__(self):
        self.reset_simulation()


class IntegrationTestInitial(IntegrationTest):
    START_PLAY = "false"

    @timeout_decorator.timeout(60 * 10)
    def test_game_start(self):
        self.team = Team(None)
        self.distance = np.inf

        def processMsg(data: RobotState):
            if data.role == RobotState.ROLE_UNASSIGNED:
                return
            coords = self.team.formations["ready"][data.role]
            self.distance = np.linalg.norm([coords[0] - data.pose.position.x, coords[1] - data.pose.position.y])

        rospy.Subscriber("/robot1/state", RobotState, processMsg)

        while not rospy.is_shutdown():
            printlog(f"Distance to destination: {self.distance}")
            rospy.sleep(1)
            if self.distance < 2:
                break
        printlog("Goal reached")


class IntegrationTestPlaying(IntegrationTest):
    START_PLAY = "true"

    # Place the ball right in front of the robot, should kick right foot
    @timeout_decorator.timeout(60 * 5)
    def test_kick_right(self):
        self.set_robot_pose(4.0, 0.0, 0)
        self.set_ball_pose(4.16, -0.04)
        while not rospy.is_shutdown():
            if self.bounding_boxes is None:
                rospy.sleep(0.1)
                continue

            gt_ball_pose = self.get_ball_pose(gt=False)
            if gt_ball_pose is not None:
                printlog(f"Current ball location: {gt_ball_pose}")
                if gt_ball_pose[0] > 4.5:
                    printlog("Goal Scored")
                    return
            else:
                printlog("Ball not found")

            rospy.sleep(2)

    @timeout_decorator.timeout(60 * 15)
    def test_walk_and_kick_right(self):
        self.set_robot_pose(3.5, 0.0, 0)
        self.set_ball_pose(4.16, -0.04)
        while not rospy.is_shutdown():
            if self.bounding_boxes is None:
                rospy.sleep(0.1)
                continue

            gt_ball_pose = self.get_ball_pose(gt=False)
            if gt_ball_pose is not None:
                printlog(f"Current ball location: {gt_ball_pose}")
                if gt_ball_pose[0] > 4.5:
                    printlog("Goal Scored")
                    return
            else:
                printlog("Ball not found")

            rospy.sleep(2)

    # Run this to generate many ball and robot locations and test that the localization are correct
    # 1. Run roslaunch soccerbot soccerbot_multi.launch competition:=false fake_localization:=true
    # If complaining missing opencv make sure LD_LIBRARY_PATH env is set
    def test_annotate_ball(self, num_samples=100, create_localization_labels=True):
        import math
        import os
        import random
        import time

        import cv2
        import numpy as np
        import rospy
        import tf
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        from tf import TransformListener

        from soccer_common.transformation import Transformation

        if not os.path.exists("soccer_object_localization/images"):
            os.makedirs("soccer_object_localization/images")

        j = 0
        for file in [
            f for f in os.listdir("soccer_object_localization/images") if os.path.isfile(os.path.join("soccer_object_localization/images", f))
        ]:
            if "border" in file:
                continue
            num = int(file.split("_")[0].replace("img", ""))
            if j < num:
                j = num

        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )

        self.camera = Camera("robot1")

        field_width = 2.5  # m
        field_height = 1.5
        ball_radius = 0.07

        while not rospy.is_shutdown() and not self.camera.ready():
            print("Waiting for camera info")

        tf_listener = TransformListener()

        for i in range(num_samples):
            robot_x = random.uniform(-field_height, field_height)
            robot_y = random.uniform(-field_width, field_width)

            robot_position = [robot_x, robot_y]
            robot_theta = random.uniform(-math.pi, math.pi)

            ball_distance_offset = random.uniform(0.8, 3)
            ball_angle_offset = random.uniform(-math.pi / 5, math.pi / 5)

            ball_offset = [
                math.cos(robot_theta + ball_angle_offset) * ball_distance_offset,
                math.sin(robot_theta + ball_angle_offset) * ball_distance_offset,
                0,
            ]

            ball_position = [ball_offset[0] + robot_position[0], ball_offset[1] + robot_position[1], 0.0783]

            print(robot_position, robot_theta)
            self.set_robot_pose(robot_position[0], robot_position[1], robot_theta)
            self.set_ball_pose(ball_position[0], ball_position[1])
            time.sleep(0.5)
            # Calculate the frame in the camera
            image_msg = rospy.wait_for_message("/robot1/camera/image_raw", Image)

            # Save the image
            rgb_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            camera_info_K = np.array(self.camera.camera_info.K).reshape([3, 3])
            camera_info_D = np.array(self.camera.camera_info.D)
            image = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

            # Annotate the image automatically
            # Set the camera
            try:
                if tf_listener.waitForTransform("world", "robot1/ball_gt", image_msg.header.stamp, rospy.Duration(1)):
                    (ball_position, rot) = tf_listener.lookupTransform("world", "robot1/ball_gt", image_msg.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException):
                rospy.logwarn_throttle(2, "Cannot find ball transform")
                continue

            self.camera.reset_position(from_world_frame=True, camera_frame="/camera_gt", timestamp=image_msg.header.stamp)
            label = self.camera.calculateBoundingBoxesFromBall(Transformation(ball_position), ball_radius)

            # Draw the rectangle
            pt1 = (round(label[0][0]), round(label[0][1]))
            pt2 = (round(label[1][0]), round(label[1][1]))
            image_rect = cv2.rectangle(image.copy(), pt1, pt2, color=(0, 0, 0), thickness=1)

            # cv2.imshow("ball", image_rect)
            # key = cv2.waitKey(0)
            # print(key)
            # if key != 32:
            #     continue

            if create_localization_labels:
                filePath = f"soccer_object_localization/images/img{j}_{robot_x}_{robot_y}_{robot_theta}.png"
                if os.path.exists(filePath):
                    os.remove(filePath)
                cv2.imwrite(filePath, image)
            else:
                filePath = f"soccer_object_localization/images/img{j}_{pt1[0]}_{pt1[1]}_{pt2[0]}_{pt2[1]}.png"
                if os.path.exists(filePath):
                    os.remove(filePath)
                cv2.imwrite(filePath, image)
                filePath2 = f"images/imgborder{j}_{pt1[0]}_{pt1[1]}_{pt2[0]}_{pt2[1]}.png"
                if os.path.exists(filePath2):
                    os.remove(filePath2)
                cv2.imwrite(filePath2, image_rect)

            j = j + 1

    def test_annotate_net(self, num_samples=100, create_localization_labels=True):
        import math
        import os
        import random
        import time

        import cv2
        import numpy as np
        import rospy
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image

        if not os.path.exists("soccer_object_localization/images"):
            os.makedirs("soccer_object_localization/images")

        j = 0
        for file in [
            f for f in os.listdir("soccer_object_localization/images") if os.path.isfile(os.path.join("soccer_object_localization/images", f))
        ]:
            if "border" in file:
                continue
            num = int(file.split("_")[0].replace("img", ""))
            if j < num:
                j = num

        os.system(
            "/bin/bash -c 'source /opt/ros/noetic/setup.bash && rosnode kill /robot1/soccer_strategy /robot1/soccer_pycontrol /robot1/soccer_trajectories'"
        )

        self.camera = Camera("robot1")

        while not rospy.is_shutdown() and not self.camera.ready():
            print("Waiting for camera info")

        for i in range(num_samples):
            robot_x = random.uniform(-4, 4)
            robot_y = random.choice([-3.15, 3.15])

            robot_position = [robot_x, robot_y]
            robot_theta = 0
            if robot_y == -3.15:
                robot_theta = random.uniform(math.pi / 2 - 1.0, math.pi / 2 + 1.0)
            elif robot_y == 3.15:
                robot_theta = random.uniform(-math.pi / 2 - 1.0, -math.pi / 2 + 1.0)

            self.set_robot_pose(robot_position[0], robot_position[1], robot_theta)
            time.sleep(0.5)
            # Calculate the frame in the camera
            image_msg = rospy.wait_for_message("/robot1/camera/image_raw", Image)

            # Save the image
            rgb_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            camera_info_K = np.array(self.camera.camera_info.K).reshape([3, 3])
            camera_info_D = np.array(self.camera.camera_info.D)
            image = cv2.undistort(rgb_image, camera_info_K, camera_info_D)

            # Annotate the image automatically
            self.camera.reset_position(from_world_frame=True, camera_frame="/camera_gt", timestamp=image_msg.header.stamp)

            if create_localization_labels:
                filePath = f"soccer_object_localization/images/img{j}_{robot_x}_{robot_y}_{robot_theta}.png"
                if os.path.exists(filePath):
                    os.remove(filePath)
                cv2.imwrite(filePath, image)

            j = j + 1
