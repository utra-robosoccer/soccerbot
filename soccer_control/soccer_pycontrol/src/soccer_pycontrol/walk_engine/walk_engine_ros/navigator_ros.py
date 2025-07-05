import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from soccer_pycontrol.model.model_ros.bez_ros import BezROS
from soccer_pycontrol.walk_engine.foot_step_planner import FootStepPlanner
from soccer_pycontrol.walk_engine.navigator import Navigator
from soccer_pycontrol.walk_engine.stabilize import Stabilize
from soccer_pycontrol.walk_engine.walker import Walker
from std_msgs.msg import Float32MultiArray

from soccer_common import PID, Transformation
from soccer_msgs.msg import BoundingBoxes, FixedTrajectoryCommand


class NavigatorRos(Navigator):
    def __init__(self, bez: BezROS, imu_feedback_enabled: bool = False, ball2: bool = False):
        self.ball = None
        self.ball2 = ball2
        self.ball_pixel = None
        self.imu_feedback_enabled = imu_feedback_enabled
        self.bez = bez

        self.foot_step_planner = FootStepPlanner(self.bez.robot_model, self.bez.parameters, self.get_time, debug=False, ball=self.ball2, sim=False)
        # TODO publish local odomtry from foot step planner
        self.rate = self.Rate(1 / self.foot_step_planner.DT)
        self.func_step = self.rate.sleep  # TODO is this needed?
        self.walker = Walker(bez, self.foot_step_planner, imu_feedback_enabled=imu_feedback_enabled)

        self.walk_pid = Stabilize(self.bez.parameters)
        self.max_vel = 0.03
        self.nav_x_pid = PID(
            Kp=0.1,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )
        self.nav_y_pid = PID(  # TODO properly tune later
            Kp=0.1,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-self.max_vel, self.max_vel),
        )  # TODO could also mod if balance is decreasing
        self.nav_yaw_pid = PID(
            Kp=0.01,
            Kd=0,
            Ki=0,
            setpoint=0,
            output_limits=(-0.1, 0.1),
        )
        self.ball_dx = 0
        self.ball_dy = 0.7
        self.error_tol = 0.05  # in m TODO add as a param and in the ros version
        self.position_create_subscription = self.create_subscription(self.bez.ns + "goal", PoseStamped, self.goal_callback)
        self.goal = PoseStamped()
        self.t = None
        self.enable_walking = None
        self.walker.reset_walk()
        self.sub_boundingbox = self.create_subscription("/robot1/ball", PoseStamped, self.box_callback)
        self.sub_ball_pixel = self.create_subscription("/robot1/ball_pixel", Float32MultiArray, self.pixel_callback)
        self.last_ball = [0, 0]
        self.last_req = self.Time.from_sec(0)
        self.ball_x_pid = PID(
            Kp=0.05,
            Kd=0,
            Ki=0.03,
            setpoint=0,
            output_limits=(-1.57, 1.57),
        )

        self.ball_y_pid = PID(
            Kp=-0.05,
            Kd=0,
            Ki=-0.03,
            setpoint=2.4,
            output_limits=(0.4, 1.3),
        )
        self.pub_all_motor = self.create_publisher("command", FixedTrajectoryCommand, queue_size=2)


    def check_request_timeout(self, nsecs: int = 500000000):
        return (self.get_clock().now() - self.last_req) < self.Duration(secs=1, nsecs=nsecs)

    def pixel_callback(self, data):
        self.ball_pixel = data.data

    def box_callback(self, data):
        self.last_req = self.get_clock().now()
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
            self.sleep(self.foot_step_planner.DT)

    def run(self, target_goal):
        angles = np.linspace(-np.pi, np.pi)
        ready = True
        kicked = False

        while not self.is_shutdown():
            # self.bez.motor_control.set_single_motor("head_pitch", 0.7)
            # self.bez.motor_control.set_single_motor("head_yaw", 0.0)
            # self.bez.motor_control.set_motor()

            if isinstance(target_goal, Transformation):
                if self.ball is not None:
                    if 0.0 < np.linalg.norm(self.ball.position[:2]) < 0.05 and kicked == False:
                        print(self.ball.position)
                        print(np.linalg.norm(self.ball.position[:2]))
                        self.ready()
                        msg = FixedTrajectoryCommand()
                        msg.trajectory_name = "rightkick"
                        msg.mirror = True
                        self.pub_all_motor.publish(msg)
                        kicked = True
                        self.walker.reset_walk()
                        self.ready()
                    elif not kicked:
                        if self.check_request_timeout():
                            ready = False
                            self.walk(self.ball,self.ball_pixel, ball_mode=True)
                        elif not ready:
                            self.ready()
                            self.bez.motor_control.set_single_motor("head_pitch", 0.7)
                            ready = True
            elif isinstance(target_goal, list):  # [d_x: float = 0.0, d_y: float = 0.0, d_theta: float = 0.0, nb_steps: int = 10, t_goal: float = 10]
                # if target_goal[:3] == [0.0,0.0,0]:
                #     if self.imu_feedback_enabled and self.bez.sensors.imu_ready:
                #         [_, pitch, roll] = self.bez.sensors.get_imu()
                #         print(pitch,"  ", roll)
                #         self.stabilize_walk(pitch, roll)
                #
                #
                #     self.bez.motor_control.configuration["head_yaw"] = self.ball_dx
                #     self.bez.motor_control.configuration["head_pitch"] = self.ball_dy
                #     self.bez.motor_control.configuration["left_elbow"] = 1.57
                #     self.bez.motor_control.configuration["right_elbow"] = 1.57
                #     self.bez.motor_control.configuration["left_shoulder_roll"] = 0.1
                #     self.bez.motor_control.configuration["right_shoulder_roll"] = 0.1
                #
                #     self.bez.motor_control.set_motor()
                # else:
                self.walk_time(target_goal)

            # print(f"Height rotation: {self.bez.sensors.get_height().orientation_euler}")
            # print(f"Height position: {self.bez.sensors.get_height().position}")

            # self.walk(target_goal, False)
            # self.walk(target_goal, True)
            # if self.bez.sensors.imu_ready:
            #     [_, pitch, roll] = self.bez.sensors.get_imu()
            #     print(pitch, "  ", roll)
            #     self.stabilize_walk(pitch, roll)
            #     self.bez.motor_control.set_motor()
            # for j in angles:
            #
            #     # print(f"POS: tf: {self.bez.sensors.get_height().position} gt:   {self.bez.sensors.get_global_height().position}")
            # print(
            #     f"Eul: tf: {self.bez.sensors.get_height().orientation_euler} gt:   {self.bez.sensors.get_global_height().orientation_euler}")
            # self.bez.motor_control.configuration["head_yaw"] = -1.57
            # self.bez.motor_control.configuration["head_pitch"] = 1.57
            # self.bez.motor_control.set_right_leg_target_angles([0,0,0,0,0,0])
            # self.bez.motor_control.set_left_leg_target_angles([0, 0, 0, 0, 0, 0])
            # set_right_leg_target_angles
            # self.bez.motor_control.set_motor()
            # if i % 1000 == 0:
            #     walk.reset_walk()

            self.func_step()
