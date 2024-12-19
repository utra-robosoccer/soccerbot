import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_pycontrol.walk_engine.stabilize import Stabilize

from soccer_common import PID, Transformation
from soccer_msgs.msg import BoundingBoxes


class NavigatorRos(Navigator):
    def __init__(self, bez: BezROS, imu_feedback_enabled: bool = False, ball2: bool = False):
        self.ball = None
        self.ball2 = ball2
        self.imu_feedback_enabled = imu_feedback_enabled
        self.bez = bez

        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, rospy.get_time, debug=False, ball=self.ball2)
        # TODO publish local odomtry from foot step planner
        self.rate = rospy.Rate(1 / self.foot_step_planner.DT)
        self.func_step = self.rate.sleep

        self.walk_pid = Stabilize(self.bez.parameters)
        self.max_vel = 0.09
        self.nav_x_pid = PID(
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.5,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.05, 0.05),
        )  # TODO could also mod if balance is decreasing
        self.nav_yaw_pid = PID(
            Kp=0.2,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.3, 0.3),
        )

        self.error_tol = 0.05  # in m TODO add as a param and in the ros version
        self.position_subscriber = rospy.Subscriber(self.bez.ns + "goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()
        self.t = None
        self.enable_walking = None
        self.reset_walk()
        self.sub_boundingbox = rospy.Subscriber("/robot1/ball", PoseStamped, self.box_callback)

    def box_callback(self, data):
        self.ball = Transformation(pose=data.pose)

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

    def run(self, target_goal):
        angles = np.linspace(-np.pi, np.pi)

        while not rospy.is_shutdown():
            self.walk(self.ball, True)
            # self.walk(target_goal, True)
            # for j in angles:
            #
            #     # print(f"POS: tf: {self.bez.sensors.get_height().position} gt:   {self.bez.sensors.get_global_height().position}")
            # print(
            #     f"Eul: tf: {self.bez.sensors.get_height().orientation_euler} gt:   {self.bez.sensors.get_global_height().orientation_euler}")
            # self.bez.motor_control.configuration["head_yaw"] = -1.57
            # self.bez.motor_control.configuration["head_pitch"] = 1.57
            # self.bez.motor_control.set_motor()
            # if i % 1000 == 0:
            #     walk.reset_walk()

            self.func_step()
