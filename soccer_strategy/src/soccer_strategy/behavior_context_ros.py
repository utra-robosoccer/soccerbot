from __future__ import annotations

import rclpy
from mavros_msgs.srv import SetMode, SetModeRequest
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

from soccer_strategy.behavior.behavior_context import BehaviorContext


class BehaviorContextRos(BehaviorContext):

    _state = None
    """
    A reference to the current state of the BehaviorContext.
    """

    def __init__(self, drone: DroneRos, sim: bool) -> None:
        self.state_list = ["land", "rtl", "takeoff"]
        # TODO decouple drone from behavior can pass from executive
        self.drone = drone
        self.sim = sim
        self.path = PathStatusRos()

        self.transition_to(DisarmRos())
        # TODO add safe guards around manual safe switching
        # Services
        self._srv_arm = self.Service("evtol_behavior/arm", Empty, self.__callback_arm)

    """
    The BehaviorContext delegates part of its behavior to the current State object.
    """

    def state_action(self) -> None:
        super(BehaviorContextRos, self).state_action()

        state_name = self._state.__class__.__name__.lower()
        # TODO add a success condition

        if state_name in self.state_list:
            self.wait_for_service("/evtol_nav/" + state_name)
            self.ServiceProxy("evtol_nav/" + state_name, Empty).call(EmptyRequest())

    @staticmethod
    def check_disarm(next_state):
        return type(next_state) == Disarm or type(next_state) == DisarmRos

    def __callback_arm(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """
        self.transition_to_arm()

        return EmptyResponse()
