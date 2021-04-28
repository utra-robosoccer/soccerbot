from wolfgang_webots_sim.webots_robot_controller import RobotController
from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController


class RobotSupervisorController(SupervisorController, RobotController):
    """Unified controller that has supervisor privileges and controls a robot"""

    def __init__(self, ros_active=False, mode='normal', robot='wolfgang', base_ns='', model_states_active=True):
        """
        A unified controller that has supervisor privileges and controls a robot.
        It inherits from SupervisorController and RobotController. Since SupervisorController is the first
        parent class, calls to super() use the SupervisorController. The methods of the controllers are mostly
        disjoint, only methods that differ in the controllers have to be reimplemented here.

        For the parameter documentation, see the parent class constructors.
        """
        super().__init__(ros_active=ros_active, mode=mode, do_ros_init=False, base_ns=base_ns, model_states_active=model_states_active)
        # This is used so that SupervisorController and RobotController can use the same underlying controller
        self.robot_node = self.supervisor
        RobotController.__init__(self, ros_active=ros_active, robot=robot, do_ros_init=False, external_controller=True, base_ns=base_ns)

    def step(self):
        super().step()
        if self.ros_active:
            RobotController.publish_ros(self)
