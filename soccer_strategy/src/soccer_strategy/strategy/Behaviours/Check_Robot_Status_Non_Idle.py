import typing

import py_trees
import rospy
from rospy import Time

from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled


class Check_Robot_Status_Non_Idle(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot: RobotControlled, time: Time):
        self.robot = robot
        self.end_of_action_time = time
        super(Check_Robot_Status_Non_Idle, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Check_Robot_Status_Non_Idle::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Check_Robot_Status_Non_Idle::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Check_Robot_Status_Non_Idle::update()]" % self.name)

        if self.robot.status in [
            Robot.Status.WALKING,
            Robot.Status.KICKING,
            Robot.Status.FALLEN_BACK,
            Robot.Status.FALLEN_FRONT,
            Robot.Status.FALLEN_SIDE,
            Robot.Status.GETTING_BACK_UP,
        ]:
            self.end_of_action_time = rospy.Time.now()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
