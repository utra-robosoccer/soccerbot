import rospy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_pycontrol.walk_engine.stabilize import Stabilize
from soccer_pycontrol.walk_engine.stabilize_nav import StabilizeNav

from soccer_common import PID


class NavigatorRos(Navigator):
    def __init__(self, bez: BezROS, imu_feedback_enabled: bool = False):
        self.imu_feedback_enabled = imu_feedback_enabled
        self.bez = bez

        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, rospy.get_time)
        # TODO publish local odomtry from foot step planner
        self.rate = rospy.Rate(1 / self.foot_step_planner.DT)
        self.func_step = self.rate.sleep

        self.walk_pid = Stabilize(self.bez.parameters)
        self.max_vel = 0.07
        self.nav_pid = StabilizeNav(self.max_vel)

        self.error_tol = 0.05  # in m TODO add as a param and in the ros version
        self.position_subscriber = rospy.Subscriber(self.bez.ns + "goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()

    def goal_callback(self, pose: PoseStamped) -> None:
        """
        Callback function for when a new goal arrives. It creates the path in the callback function and dynamically
        updates the current path if it exists (currently not working). Note the path computation occurs here
        instead of run because the computation will disturb the main thread. Main thread should still be running

        :param pose: The pose sent by the strategy for the robot to go to
        """

        self.goal = pose
        self.foot_step_planner.configure_planner(d_x=0.03)

    def wait(self, steps: int):
        for i in range(steps):
            rospy.sleep(self.foot_step_planner.DT)
