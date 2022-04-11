import os
import time
from unittest import TestCase
from soccer_msgs.msg import RobotState
from soccer_strategy.src.team import Team
import numpy as np
from robot import Robot

import rospy


DIST_TOLERANCE = 2


class IntegrationTest(TestCase):
    def setUp(self) -> None:
        super().setUp()
        os.system("docker-compose down")
        os.system("docker-compose up -d")
        time.sleep(15)
        rospy.init_node("test_integration")
        self.team = Team(None)
        self.distance = np.inf


    def tearDown(self) -> None:
        super().tearDown()
        os.system("docker-compose stop")


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


    def test_validate_game(self):
        while not rospy.is_shutdown():
            # Validate that the robot publishes robot????
            rospy.sleep(1)
            try:
                rospy.wait_for_message('/robot1/state', RobotState, 10)
            except rospy.ROSException:
                print("Connection Failed")


            handle = rospy.Subscriber("/robot1/state", RobotState, self.processMsg)
            rospy.wait_for_message('/robot1/state', RobotState, 120)
            assert handle.get_num_connections() > 0
            print("Connection looks okay")
            # Validate that the robot moves towards the goal
            for i in range(0, 100): # 100s timeout
                print("dist: {}, cycle: {}".format(self.distance, i))
                rospy.sleep(1)
                if self.distance < DIST_TOLERANCE:
                    break
            assert self.distance < DIST_TOLERANCE
            print("Goal reached")


            # Validates that the robot kicks the ball
            # TODO
            break