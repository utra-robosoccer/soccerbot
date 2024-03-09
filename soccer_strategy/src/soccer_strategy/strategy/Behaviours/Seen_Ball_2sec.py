import typing

import py_trees
import rospy

from soccer_strategy.team import Team


class Seen_Ball_2sec(py_trees.behaviour.Behaviour):
    def __init__(self, name, friendly_team: Team):
        self._friendly_team = friendly_team
        super(Seen_Ball_2sec, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Seen_Ball_2sec::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Seen_Ball_2sec::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Seen_Ball_2sec::update()]" % self.name)

        if self._friendly_team.observed_ball is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
