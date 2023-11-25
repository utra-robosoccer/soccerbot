import typing

import py_trees
from rospy import Time

from soccer_strategy.team import Team


class Have_seen_ball(py_trees.behaviour.Behaviour):
    def __init__(self, name, friendly: Team, time: Time):
        self.friendly_team = friendly
        self.end_of_action_time = time
        super(Have_seen_ball, self).__init__(name)

    def setup(self, **kwargs: typing.Any):
        self.logger.debug(" %s [Have_seen_ball::setup()]" % self.name)

    def initialise(self) -> None:
        self.logger.debug(" %s [Have_seen_ball::initalise()]" % self.name)

    def update(self):
        self.logger.debug(" %s [Have_seen_ball::update()]" % self.name)

        if self.friendly_team.observed_ball is not None:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE
