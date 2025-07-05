#!/usr/bin/env python3
import os

import rclpy
from evtol_behavior.autopilot_context_ros import AutoPilotContextRos
from evtol_behavior.behavior.state.land import Land
from evtol_behavior.behavior_context_ros import BehaviorContextRos
from evtol_behavior.health_system import HealthSystem
from evtol_common.drone_ros import DroneRos  # type: ignore[attr-defined]
from evtol_msgs.msg import DroneStatus  # type: ignore[attr-defined]
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolRequest


class BehaviorExecutiveRos:
    """
    This class is responsible for the main decision-making of the evtol and uses all systems to control the evtol.
    Integration and delegation for other modules. Code for any decision drone has to make.
    """

    def __init__(self):
        # Initialize node
        self.init_node("evtol_behavior")

        # Initialize attributes
        # TODO tests fail if 30 hz, that shouldnt hapen
        self.rate = self.Rate(self.get_param("/evtol_nav/rate", 20))
        self.sim = self.get_param("/simulation", os.environ.get("SIM", False))
        self.last_req = self.get_clock().now()

        self._drone = DroneRos()
        # TODO limit drone nand path to pass from here during loop
        self._behavior = BehaviorContextRos(self._drone, self.sim)  # TODO clean up
        self._autopilot = AutoPilotContextRos(self._behavior)
        self._health_system = HealthSystem()

        # create_publisher
        self._drone_status_pub = self.create_publisher("evtol_behavior/drone_status", DroneStatus, queue_size=10)

        # TODO add uwb to sim
        if not self.sim:
            self.wait_for_service("/evtol_sensors/uwb/enable")
            self.srv = self.ServiceProxy("evtol_sensors/uwb/enable", SetBool)

    # Main communication node for ground control
    def run(self):
        """
        Main loop

        :return: None
        """

        # Main loop to follow waypoints
        while not self.is_shutdown():
            # Behaviour Executive
            # TODO pass drone & path harder then previously thought might be possible but not worth time rigth now
            self._behavior.run_state_algorithim()

            # Health System TODO fix starting issue and maybe but this
            if self._health_system.check_health(self._drone.z - self._drone.disarm_height):
                if not self.sim:
                    self.srv.call(SetBoolRequest(data=True))
                    self.get_logger().error(1, "switching to uwb")
                self._behavior.state = Land()

            # AutoPilot
            # TODO pass behavior
            self._autopilot.check_autopilot()

            # TODO put in drone maybe?
            msg = DroneStatus(header=Header(stamp=self.get_clock().now(), frame_id="map"), data=self._drone.status)
            self._drone_status_pub.publish(msg)

            self.rate.sleep()
