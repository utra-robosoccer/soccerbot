import rospy
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse


class AutoPilotContextRos(AutoPilotContext):
    def __init__(self, behavior: BehaviorContextRos) -> None:
        """
        Usually, the AutoPilotContext accepts an autopilot through the constructor, but
        also provides a setter to change it at runtime.
        """
        super(AutoPilotContextRos, self).__init__(AutoPilotActionsRos(behavior))

        self._srv_hover = rospy.Service("evtol_behavior/test/hover", Empty, self.__callback_hover)
        self._srv_hover_kill = rospy.Service("evtol_behavior/test/hover_kill", Empty, self.__callback_hover_kill)
        self._srv_hover_uwb = rospy.Service("evtol_behavior/test/hover_uwb", Empty, self.__callback_hover_uwb)
        self._srv_alt = rospy.Service("evtol_behavior/test/altitude", Empty, self.__callback_altitude)
        self._srv_square_hover = rospy.Service("evtol_behavior/test/square_hover", Empty, self.__callback_square_hover)
        self._srv_square_traj = rospy.Service("evtol_behavior/test/square_traj", Empty, self.__callback_square_traj)
        self._srv_circle_traj = rospy.Service("evtol_behavior/test/circle_traj", Empty, self.__callback_circle_traj)
        self._srv_lemniscate_traj = rospy.Service("evtol_behavior/test/lemniscate_traj", Empty, self.__callback_lemniscate_traj)

        # TODO can i add the same trick i used in behaviorcontextros

    def __callback_hover(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the hover test service.
        """
        rospy.loginfo("hover")

        self.autopilot = Hover(self.action)
        self.autopilot.inprogress = True

        return EmptyResponse()

    def __callback_hover_kill(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the hover test service.
        """
        rospy.loginfo("hover kill")

        self.autopilot = HoverKill(self.action)
        self.autopilot.inprogress = True

        return EmptyResponse()

    def __callback_hover_uwb(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the hover test service.
        """
        rospy.loginfo("hover uwb")

        self.autopilot = HoverUwb(self.action)
        self.autopilot.inprogress = True

        return EmptyResponse()

    def __callback_altitude(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """

        self.autopilot = Altitude(self.action)
        self.autopilot.inprogress = True
        return EmptyResponse()

    def __callback_square_hover(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """

        self.autopilot = SquareHover(self.action)
        self.autopilot.inprogress = True
        return EmptyResponse()

    def __callback_square_traj(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """

        self.autopilot = Trajectory(self.action, "square")
        self.autopilot.inprogress = True
        return EmptyResponse()

    def __callback_circle_traj(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """
        self.autopilot = Trajectory(self.action, "circle")
        self.autopilot.inprogress = True
        return EmptyResponse()

    def __callback_lemniscate_traj(self, request: EmptyRequest) -> EmptyResponse:
        """
        This function handles the arm service, which triggers the arming action of the evtol.
        """
        self.autopilot = Trajectory(self.action, "lemniscate")
        self.autopilot.inprogress = True
        return EmptyResponse()
