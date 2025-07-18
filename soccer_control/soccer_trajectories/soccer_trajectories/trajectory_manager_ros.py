#!/usr/bin/env python3
import os
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from soccer_trajectories.trajectory_manager import TrajectoryManager
from std_msgs.msg import Bool, Empty

from soccer_common import Transformation
from soccer_msgs.msg import FixedTrajectoryCommand, RobotState

# if "ROS_NAMESPACE" not in os.environ:
#     os.environ["ROS_NAMESPACE"] = "/robot1"


class TrajectoryManagerRos(TrajectoryManager, Node):
    """
    Interfaces with trajectory and manages interaction with ROS
    """

    # TODO entire module needs a rewrite

    def __init__(self, robot_model: str = "bez2", trajectory_name: str = "rightkick"):
        Node.__init__(self, "soccer_trajectories")

        self.declare_parameter("sim", os.environ.get("SIM", False))
        sim = self.get_parameter("sim").get_parameter_value().bool_value
        use_sim_time_prefix = "_sim" if sim == "true" else ""
        # TODO fix with param later
        path = os.path.join(os.path.dirname(__file__), "../../trajectories/") + "bez2"  # + use_sim_time_prefix

        # TODO fix later
        super(TrajectoryManagerRos, self).__init__(robot_model, trajectory_name)
        self.imu_msg = Imu()
        self.imu_ready = True
        self.imu_create_subscription = self.create_subscription(Imu, "imu_filtered", self.imu_callback, qos_profile=10)

        self.terminate = False
        self.period = 100  # Maybe 200 Hz

        self.command_create_subscription = self.create_subscription(FixedTrajectoryCommand, "command", self.command_callback, qos_profile=10)
        # self.robot_state_create_subscription = self.create_subscription(RobotState,"state",  self.robot_state_callback, qos_profile=10)
        self.joint_state_subscription = self.create_subscription(JointState, "joint_states", self.joint_callback, qos_profile=10)
        self.last_joint_state = JointState()
        self.last_joint_state_req = self.get_clock().now()
        ns = ""
        self.pub_all_motor = self.create_publisher(JointState, "joint_command", qos_profile=10)
        self.pub_all_motor2 = self.create_publisher(JointState, ns + "joint_command", qos_profile=10)
        self.finish_trajectory_create_publisher = self.create_publisher(Empty, "action_complete", qos_profile=10)

        # self._timer = self.create_timer(1 / self.period, self.run)

        self.rate = self.create_rate(150)
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()
        self.traj_in_progress = False
        self.traj_prog = self.create_publisher(Bool, "traj_prog", qos_profile=10)
        # self.get_logger().info("fdfs")

    # def robot_state_callback(self, state: RobotState): # TODO reenable when at that part
    #     if state.status in [RobotState.STATUS_PENALIZED]:
    #         if self.trajectory.max_time > 0:
    #             self.terminate = True
    def joint_callback(self, msg: JointState):
        self.last_joint_state = msg

    def imu_callback(self, msg: Imu):
        """
        Callback function for IMU information

        :param msg: IMU Message
        """
        self.imu_msg = msg
        self.imu_ready = True

    def get_imu(self):
        """
        Gets the IMU at the IMU link location.

        :return: calculated orientation of the center of the torso of the robot
        """

        assert self.imu_ready
        return Transformation(
            [0, 0, 0],
            [
                self.imu_msg.orientation.x,
                self.imu_msg.orientation.y,
                self.imu_msg.orientation.z,
                self.imu_msg.orientation.w,
            ],
        ).orientation_euler

    def command_callback(self, command: FixedTrajectoryCommand):
        # TODO Maybe only dont interrupt for other reasons
        self.traj_in_progress = True
        self.traj_prog.publish(Bool(data=self.traj_in_progress))
        if self.trajectory.max_time > 0:
            return

        # path = self.trajectory_path + command.trajectory_name + ".csv"
        #
        # if not os.path.exists(path):
        #     self.get_logger().error(f"Trajectory doesn't exist in path {path}")
        #     return

        self.process_trajectory(command.trajectory_name, command.mirror)

    def process_trajectory(self, path: str, mirror: bool):
        self.get_logger().info("Running Trajectory: " + path + f" {mirror}")
        super(TrajectoryManagerRos, self).process_trajectory(path, mirror)

    # self.get_logger().info(f"Lost connection to serial port {ex}, retrying...", throttle_duration_sec=1)
    def read_joint_state(self) -> JointState:
        last_joint_state = JointState()

        if bool((self.get_clock().now().to_msg().sec - self.last_joint_state.header.stamp.sec) < 2):
            last_joint_state = self.last_joint_state
        else:
            self.get_logger().error("Joint state > 2 sec")

        return last_joint_state

    def send_joint_msg(self, timestamp: float) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        joints, angles = self.trajectory.create_joint_states(timestamp)
        print(joints, angles)
        js.name = joints
        js.position = angles
        self.pub_all_motor2.publish(js)

    def send_trajectory(self, real_time: bool = True) -> None:
        t: float = 0
        while rclpy.ok() and t <= self.trajectory.max_time and not self.terminate:

            try:
                # if t % 0.5:
                #     self.get_logger().info("2")
                self.send_joint_msg(t)
            except Exception as ex:  # TODO find a proper exepction
                print(ex)
                self.destroy_node()
                rclpy.shutdown()
                exit(0)
            t += 1.0 / self.period
            if int(t + (1.0 / self.period)) != int(t):
                print(f"Trajectory at t={t}")
            # t += 0.01
            # if int(t + 0.01) != int(t):
            #     print(f"Trajectory at t={t}")
            if real_time:
                # self.get_logger().info("3")
                self.rate.sleep()
                # self.get_logger().info("4")
        self.traj_in_progress = False
        self.traj_prog.publish(Bool(data=self.traj_in_progress))
        self.get_logger().info("Finished Trajectory: " + self.trajectory.trajectory_path)
        self.finish_trajectory_create_publisher.publish(Empty())
        self.trajectory.reset()

    def run(self) -> None:
        # TODO could be in cmd with a spin

        while rclpy.ok():
            [_, p, r] = self.get_imu()

            if p > 1.25:
                print("getupfront")
                msg = FixedTrajectoryCommand()
                msg.trajectory_name = "getupfront"
                msg.mirror = False
                self.command_callback(command=msg)
                self.send_trajectory(real_time=True)
            elif p < -1.25:
                print("getupback: ")
                msg = FixedTrajectoryCommand()
                msg.trajectory_name = "getupback"
                msg.mirror = False
                self.command_callback(command=msg)
                self.send_trajectory(real_time=True)
            elif r < -1.54 and -0.5 < p < -0.4:
                print("getupback: ")
                msg = FixedTrajectoryCommand()
                msg.trajectory_name = "getupside"
                msg.mirror = False
                self.command_callback(command=msg)
                self.send_trajectory(real_time=True)
                # tm.send_trajectory("getupsideleft")
            elif r > 1.54 and -0.5 < p < -0.4:
                msg = FixedTrajectoryCommand()
                msg.trajectory_name = "getupside"
                msg.mirror = False
                self.command_callback(command=msg)
                self.send_trajectory(real_time=True)
            # if self.trajectory.max_time > 0:
            #     msg = FixedTrajectoryCommand()
            #     msg.trajectory_name = "getupback"
            #     # msg.trajectory_name = "getupside"
            #     # msg.trajectory_name = "getupfront_ori"
            #     msg.mirror = False

            # self.send_trajectory()

            self.rate.sleep()


def main():
    rclpy.init()
    node = TrajectoryManagerRos()
    msg = FixedTrajectoryCommand()
    msg.trajectory_name = "getupback"
    # msg.trajectory_name = "getupside"
    # msg.trajectory_name = "getupfront_ori"
    msg.mirror = False
    try:
        # rclpy.spin(node)
        node.run()
        # node.rate.sleep()
        # node.command_callback(command=msg)
        # node.send_trajectory(real_time=True)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
