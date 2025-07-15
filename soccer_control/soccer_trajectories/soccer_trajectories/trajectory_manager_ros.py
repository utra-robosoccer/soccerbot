#!/usr/bin/env python3
import os

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from soccer_trajectories.trajectory_manager import TrajectoryManager
from std_msgs.msg import Empty

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
        path = os.path.join(os.path.dirname(__file__), "../../trajectories/") + "bez2" + use_sim_time_prefix

        # TODO fix later
        super(TrajectoryManagerRos, self).__init__(robot_model, trajectory_name)

        self.terminate = False
        self.period = 100  # Maybe 200 Hz

        self.command_create_subscription = self.create_subscription(FixedTrajectoryCommand, "command", self.command_callback, qos_profile=10)
        # self.robot_state_create_subscription = self.create_subscription(RobotState,"state",  self.robot_state_callback, qos_profile=10)
        self.joint_state_subscription = self.create_subscription(JointState, "joint_states", self.joint_callback, qos_profile=10)
        self.last_joint_state = JointState()
        self.last_joint_state_req = self.get_clock().now()

        self.pub_all_motor = self.create_publisher(JointState, "joint_command", qos_profile=10)
        self.finish_trajectory_create_publisher = self.create_publisher(Empty, "action_complete", qos_profile=10)

        self._timer = self.create_timer(1 / self.period, self.run)

    # def robot_state_callback(self, state: RobotState): # TODO reenable when at that part
    #     if state.status in [RobotState.STATUS_PENALIZED]:
    #         if self.trajectory.max_time > 0:
    #             self.terminate = True
    def joint_callback(self, msg: JointState):
        self.last_joint_state = msg

    def command_callback(self, command: FixedTrajectoryCommand):
        # TODO Maybe only dont interrupt for other reasons
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
        self.pub_all_motor.publish(js)

    def send_trajectory(self, real_time: bool = True) -> None:
        t: float = 0
        while rclpy.ok() and t <= self.trajectory.max_time and not self.terminate:
            try:
                # if t % 0.5:
                #     print(1)
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

        self.get_logger().info("Finished Trajectory: " + self.trajectory.trajectory_path)
        self.finish_trajectory_create_publisher.publish(Empty())
        self.trajectory.reset()

    def run(self) -> None:
        # TODO could be in cmd with a spin
        # while rclpy.ok():
        if self.trajectory.max_time > 0:
            self.send_trajectory()


def main():
    rclpy.init()
    node = TrajectoryManagerRos()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
