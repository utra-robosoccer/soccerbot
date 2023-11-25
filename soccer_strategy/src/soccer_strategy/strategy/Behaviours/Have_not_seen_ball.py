import math
import typing

import numpy as np
import py_trees
import rospy
from rospy import Time

from soccer_common.utils import wrapTo2Pi
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled
from soccer_strategy.team import Team


class Have_not_seen_ball(py_trees.behaviour.Behaviour):
    def __init__(self, name, friendly: Team, time: Time, robot: RobotControlled):
        self.friendly = friendly
        self.end_of_action_time = time
        self.this_robot = robot
        super(Have_not_seen_ball, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Have_not_seen_ball::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Have_not_seen_ball::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Have_not_seen_ball::update()]" % self.name)

        if (
            self.friendly.observed_ball is None
            or rospy.Time.now() - self.friendly.observed_ball.last_observed_time_stamp
            > rospy.Duration(rospy.get_param("delay_before_rotate_to_search_ball", 10))
            and rospy.Time.now() - self.end_of_action_time > rospy.Duration(rospy.get_param("delay_before_rotate_to_search_ball", 10))
        ):
            if self.this_robot.status not in [Robot.Status.WALKING, Robot.Status.KICKING]:
                player_angle = self.this_robot.position[2]
                player_position = self.this_robot.position[0:2]

                # Haven't seen the ball timeout
                rospy.loginfo(
                    f"Player {self.this_robot.robot_id}: Rotating to locate ball. Time of End of Action "
                    f"{self.end_of_action_time}, Last Observed Time Stamp "
                    f"{self.friendly.observed_ball.last_observed_time_stamp if self.friendly.observed_ball is not None else 0}"
                )

                turn_position = np.array([player_position[0], player_position[1], wrapTo2Pi(player_angle + math.pi * 0.45)])
                self.this_robot.set_navigation_position(turn_position)
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
