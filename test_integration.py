import os
from unittest import TestCase

import rospy


class Test(TestCase):
    def setUp(self) -> None:
        super().setUp()
        os.system("docker-compose up -d")
        rospy.init_node("test_integration")

    def tearDown(self) -> None:
        super().tearDown()
        os.system("docker-compose stop")

    def test_validate_game(self):
        while not rospy.is_shutdown():
            # Validate that the robot publishes robot state

            # Validate that the robot moves towards the goal

            # Validates that the robot kicks the ball

            rospy.sleep(1)
