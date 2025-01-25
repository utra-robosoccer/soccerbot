#!/usr/bin/env python3
import os

import rospy
from evtol_behavior.autopilot_context_ros import AutoPilotContextRos
from evtol_behavior.behavior.state.land import Land
from evtol_behavior.behavior_context_ros import BehaviorContextRos
from evtol_behavior.health_system import HealthSystem
from evtol_common.drone_ros import DroneRos  # type: ignore[attr-defined]
from evtol_msgs.msg import DroneStatus  # type: ignore[attr-defined]
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolRequest


class BehaviorExecutive:
    """
    This class is responsible for the main decision-making of the evtol and uses all systems to control the evtol.
    Integration and delegation for other modules. Code for any decision drone has to make.
    """

    def __init__(self):
        # Initialize node
        rospy.init_node("evtol_behavior")

        # Initialize attributes
        # TODO tests fail if 30 hz, that shouldnt hapen
        self.rate = rospy.Rate(rospy.get_param("/evtol_nav/rate", 20))
        self.sim = rospy.get_param("/simulation", os.environ.get("SIM", False))
        self.last_req = rospy.Time.now()

        self._drone = DroneRos()
        # TODO limit drone nand path to pass from here during loop
        self._behavior = BehaviorContextRos(self._drone, self.sim)  # TODO clean up
        self._autopilot = AutoPilotContextRos(self._behavior)
        self._health_system = HealthSystem()

        # Publisher
        self._drone_status_pub = rospy.Publisher("evtol_behavior/drone_status", DroneStatus, queue_size=10)

        # TODO add uwb to sim
        if not self.sim:
            rospy.wait_for_service("/evtol_sensors/uwb/enable")
            self.srv = rospy.ServiceProxy("evtol_sensors/uwb/enable", SetBool)

    # Main communication node for ground control
    def run(self):
        """
        Main loop

        :return: None
        """

        # Main loop to follow waypoints
        while not rospy.is_shutdown():
            # Behaviour Executive
            # TODO pass drone & path harder then previously thought might be possible but not worth time rigth now
            self._behavior.run_state_algorithim()

            # Health System TODO fix starting issue and maybe but this
            if self._health_system.check_health(self._drone.z - self._drone.disarm_height):
                if not self.sim:
                    self.srv.call(SetBoolRequest(data=True))
                    rospy.loginfo_throttle(1, "switching to uwb")
                self._behavior.state = Land()

            # AutoPilot
            # TODO pass behavior
            self._autopilot.check_autopilot()

            # TODO put in drone maybe?
            msg = DroneStatus(header=Header(stamp=rospy.Time.now(), frame_id="map"), data=self._drone.status)
            self._drone_status_pub.publish(msg)

            self.rate.sleep()
