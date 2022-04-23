import os
import signal
import subprocess
import sys
from unittest import TestCase

import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose
from robot import Robot
from rosgraph_msgs.msg import Clock
from scipy.spatial.transform import Rotation as R
from timeout_decorator import timeout_decorator

from soccer_common.camera import Camera
from soccer_common.transformation import Transformation
from soccer_msgs.msg import RobotState
from soccer_object_detection.msg import BoundingBoxes
from soccer_strategy.src.team import Team

RUN_LOCALLY = "pycharm" in sys.argv[0]


class IntegrationTest(TestCase):
    START_PLAY = "false"

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
                    f"export START_PLAY={self.START_PLAY} && source ~/catkin_ws/devel/setup.bash && roslaunch soccerbot soccerbot_multi.launch",
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
        self.clock_pub = rospy.Publisher("/clock_test", Clock)

    def clock_callback(self, c: Clock):
        c.clock.secs += 1
        self.clock_pub.publish(c)

    def bounding_boxes_callback(self, b: BoundingBoxes):
        self.bounding_boxes = b

    def set_robot_pose(self, x, y, theta):
        resetPublisher = rospy.Publisher("/robot1/reset_robot", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        r = R.from_euler("ZYX", [theta, 0, 0], degrees=False)
        q = r.as_quat()

        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        resetPublisher.publish(p)

    def set_ball_pose(self, x, y):
        ballPublisher = rospy.Publisher("/reset_ball", Pose, queue_size=1, latch=True)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0

        ballPublisher.publish(p)

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

    def test_game_start(self):
        self.team = Team(None)
        self.distance = np.inf

        DIST_TOLERANCE = 2

        def processMsg(self, data: RobotState):
            print("processMsg, status {}, role {}".format(data.status, data.role))
            if data.role == RobotState.ROLE_GOALIE:
                coords = self.team.formations["ready"][Robot.Role.GOALIE]
                print("Robot.Role.GOALIE - {}".format(coords))
            elif data.role == RobotState.ROLE_LEFT_WING:
                coords = self.team.formations["ready"][Robot.Role.LEFT_WING]
                print("Robot.Role.LEFT_WING - {}".format(coords))
            elif data.role == RobotState.ROLE_RIGHT_WING:
                coords = self.team.formations["ready"][Robot.Role.RIGHT_WING]
                print("Robot.Role.RIGHT_WING - {}".format(coords))
            elif data.role == RobotState.ROLE_STRIKER:
                coords = self.team.formations["ready"][Robot.Role.STRIKER]
                print("Robot.Role.STRIKER - {}".format(coords))
            self.distance = np.linalg.norm([coords[0] - data.pose.position.x, coords[1] - data.pose.position.y])
            print("processMsg distance: {}".format(self.distance))

        while not rospy.is_shutdown():
            # Validate that the robot publishes robot????
            rospy.sleep(1)
            try:
                rospy.wait_for_message("/robot1/state", RobotState, 10)
            except rospy.ROSException:
                print("Connection Failed")

            handle = rospy.Subscriber("/robot1/state", RobotState, self.processMsg)
            rospy.wait_for_message("/robot1/state", RobotState, 120)
            assert handle.get_num_connections() > 0
            print("Connection looks okay")
            # Validate that the robot moves towards the goal
            for i in range(0, 100):  # 100s timeout
                print("dist: {}, cycle: {}".format(self.distance, i))
                rospy.sleep(1)
                if self.distance < DIST_TOLERANCE:
                    break
            assert self.distance < DIST_TOLERANCE
            print("Goal reached")

            # Validates that the robot kicks the ball
            # TODO
            break


class IntegrationTestPlaying(IntegrationTest):
    START_PLAY = "true"

    # Place the ball right in front of the robot, should kick right foot
    @timeout_decorator.timeout(10000)
    def test_kick_right(self):
        self.set_robot_pose(3.5, 0, 0)
        self.set_ball_pose(3.69, -0.04)
        while not rospy.is_shutdown():
            if self.bounding_boxes is None:
                rospy.sleep(0.1)
                continue

            try:
                t = self.tf_listener.getLatestCommonTime("/world", "/robot1/ball_gt")
                (trans_ball_gt, rot) = self.tf_listener.lookupTransform("/world", "/robot1/ball_gt", t)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.camera.reset_position(publish_basecamera=False, from_world_frame=True, timestamp=t)
            bounding_box_gt = self.camera.calculateBoundingBoxesFromBall(Transformation(trans_ball_gt), ball_radius=0.07)
            print(bounding_box_gt)
            print(self.bounding_boxes)

            try:
                (trans_ball, rot) = self.tf_listener.lookupTransform("/world", "/robot1/ball", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            # distance_to_gt = np.linalg.norm(trans_ball_gt - trans_ball)
            # self.assertTrue(distance_to_gt < 0.05) # Assert 5 percent distance
