import typing

import py_trees
import rospy
from rospy import Time

from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled


class Not_In_Action(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot: RobotControlled, time: Time):
        self.robot = robot
        self.end_of_action_time = time
        super(Not_In_Action, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Not_In_Action::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Not_In_Action::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Not_In_Action::update()]" % self.name)

        if self.robot.status in [
            Robot.Status.WALKING,
            Robot.Status.KICKING,
            Robot.Status.FALLEN_BACK,
            Robot.Status.FALLEN_FRONT,
            Robot.Status.FALLEN_SIDE,
            Robot.Status.GETTING_BACK_UP,
        ]:
            self.time = rospy.Time.now()
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
