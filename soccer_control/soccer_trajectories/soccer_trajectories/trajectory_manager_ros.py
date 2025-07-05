#!/usr/bin/env python3
import os

from sensor_msgs.msg import JointState
from soccer_trajectories.trajectory_manager import TrajectoryManager

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty

from soccer_msgs.msg import FixedTrajectoryCommand, RobotState

class TrajectoryManagerRos(TrajectoryManager, Node):
    """
    Interfaces with trajectory and manages interaction with ROS
    """

    def __init__(self, robot_model: str = "bez2", trajectory_name: str ="rightkick"):
        Node.__init__(self, "soccer_trajectories")


        use_sim_time_prefix = "_sim" if self.get_param("use_sim_time", "false") == "true" else ""
        # TODO fix
        path = (
            self.get_param_cached(
                "~trajectory_path", os.path.join(os.path.dirname(__file__), "../../trajectories/") + self.get_param_cached("robot_model", "bez1")
            )
            + use_sim_time_prefix
        )
        # TODO fix later
        super(TrajectoryManagerRos, self).__init__(robot_model, trajectory_name)

        self.terminate = False
        self.period = 100
        self.rate = self.Rate(self.period)

        self.command_create_subscription = self.create_subscription("command", FixedTrajectoryCommand, self.command_callback, queue_size=1)
        self.robot_state_create_subscription = self.create_subscription("state", RobotState, self.robot_state_callback, queue_size=1)

        self.pub_all_motor = self.create_publisher("joint_command", JointState, queue_size=2)
        self.finish_trajectory_create_publisher = self.create_publisher("action_complete", Empty, queue_size=1)

    def robot_state_callback(self, state: RobotState):
        if state.status in [RobotState.STATUS_PENALIZED]:
            if self.trajectory.max_time > 0:
                self.terminate = True

    def command_callback(self, command: FixedTrajectoryCommand):
        # TODO Maybe only dont interrupt for other reasons
        if self.trajectory.max_time > 0:
            return

        path = self.trajectory_path + command.trajectory_name + ".csv"

        if not os.path.exists(path):
            self.logerr(f"Trajectory doesn't exist in path {path}")
            return

        self.process_trajectory(path, command.mirror)

    def process_trajectory(self, path: str, mirror: bool):
        self.loginfo("Running Trajectory: " + path + f" {mirror}")
        super(TrajectoryManagerRos, self).process_trajectory(path, mirror)

    def read_joint_state(self) -> JointState:
        last_joint_state = JointState()
        try:
            last_joint_state = self.wait_for_message("joint_states", JointState, timeout=2)
        except (ROSException, AttributeError) as ex:
            self.logerr(ex)

        return last_joint_state

    def send_joint_msg(self, timestamp: float) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now()
        joints, angles = self.trajectory.create_joint_states(timestamp)
        js.name = joints
        js.position = angles
        self.pub_all_motor.publish(js)

    def send_trajectory(self, real_time: bool = True) -> None:
        t: float = 0
        while not self.is_shutdown() and t <= self.trajectory.max_time and not self.terminate:
            try:
                if t % 0.5:
                    print(1)
                self.send_joint_msg(t)
            except ROSException as ex:
                print(ex)
                exit(0)
            t += 1.0 / self.period
            if int(t + (1.0 / self.period)) != int(t):
                print(f"Trajectory at t={t}")
            # t += 0.01
            # if int(t + 0.01) != int(t):
            #     print(f"Trajectory at t={t}")
            if real_time:
                self.rate.sleep()

        self.loginfo("Finished Trajectory: " + self.trajectory.trajectory_path)
        self.finish_trajectory_create_publisher.publish()
        self.trajectory.reset()

    def run(self) -> None:
        # TODO could be in cmd with a spin
        while not self.is_shutdown():
            if self.trajectory.max_time > 0:
                trajectory_class.send_trajectory()

            self.rate.sleep()


if __name__ == "__main__":
    self.init_node("soccer_trajectories")
    trajectory_class = TrajectoryManagerRos()
    try:
        trajectory_class.run()
    except ROSException as ex:
        print(ex)
