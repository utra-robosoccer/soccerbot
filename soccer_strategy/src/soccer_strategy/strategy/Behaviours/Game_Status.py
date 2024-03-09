import typing

import py_trees
import rospy
from rospy import Time

from soccer_msgs.msg import GameState
from soccer_strategy.robot import Robot


class Game_Status(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot: Robot, game_status: GameState):
        self._game_status = game_status
        self._robot = robot
        super(Game_Status, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Game_Status::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Game_Status::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Game_Status::update()]" % self.name)

        if self._game_status.secondary_seconds_remaining != 0:
            if not self._game_status.hasKickOff:
                rospy.loginfo_throttle(15, f"Team {self._robot.team.name} waiting for other team to kickoff")
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
