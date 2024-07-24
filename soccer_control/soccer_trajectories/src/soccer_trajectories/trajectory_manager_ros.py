#!/usr/bin/env python3
import os

from rospy import ROSException
from sensor_msgs.msg import JointState
from soccer_trajectories.trajectory_manager import TrajectoryManager

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rospy
from std_msgs.msg import Empty

from soccer_msgs.msg import FixedTrajectoryCommand, RobotState


class TrajectoryManagerRos(TrajectoryManager):
    """
    Interfaces with trajectory and manages interaction with ROS
    """

    def __init__(self):
        use_sim_time_prefix = "_sim" if rospy.get_param("use_sim_time", "false") == "true" else ""
        # TODO fix
        path = (
            rospy.get_param_cached(
                "~trajectory_path", os.path.join(os.path.dirname(__file__), "../../trajectories/") + rospy.get_param_cached("robot_model", "bez1")
            )
            + use_sim_time_prefix
        )
        # TODO fix later
        super(TrajectoryManagerRos, self).__init__(path)

        self.terminate = False

        self.rate = rospy.Rate(100)

        self.command_subscriber = rospy.Subscriber("command", FixedTrajectoryCommand, self.command_callback, queue_size=1)
        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.robot_state_callback, queue_size=1)

        self.pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=2)
        self.finish_trajectory_publisher = rospy.Publisher("action_complete", Empty, queue_size=1)

    def robot_state_callback(self, state: RobotState):
        if state.status in [RobotState.STATUS_PENALIZED]:
            if self.trajectory.max_time > 0:
                self.terminate = True

    def command_callback(self, command: FixedTrajectoryCommand):
        # TODO Maybe only dont interrupt for other reasons
        if self.trajectory.max_time > 0:
            return

        path = self.trajectory_path + "/" + command.trajectory_name + ".csv"

        if not os.path.exists(path):
            rospy.logerr(f"Trajectory doesn't exist in path {path}")
            return

        self.process_trajectory(path, command.mirror)

    def process_trajectory(self, path: str, mirror: bool):
        rospy.loginfo("Running Trajectory: " + path + f" {mirror}")
        super(TrajectoryManagerRos, self).process_trajectory(path, mirror)

    def read_joint_state(self) -> JointState:
        last_joint_state = JointState()
        try:
            last_joint_state = rospy.wait_for_message("joint_states", JointState, timeout=2)
        except (ROSException, AttributeError) as ex:
            rospy.logerr(ex)

        return last_joint_state

    def send_joint_msg(self, timestamp: float) -> None:
        js = JointState()
        js.header.stamp = rospy.Time.now()
        joints, angles = self.trajectory.create_joint_states(timestamp)
        js.name = joints
        js.position = angles
        self.pub_all_motor.publish(js)

    def send_trajectory(self, real_time: bool = True) -> None:
        t: float = 0
        while not rospy.is_shutdown() and t <= self.trajectory.max_time and not self.terminate:
            try:
                self.send_joint_msg(t)
            except ROSException as ex:
                print(ex)
                exit(0)
            t += 0.01
            if int(t + 0.01) != int(t):
                print(f"Trajectory at t={t}")

            if real_time:
                self.rate.sleep()

        rospy.loginfo("Finished Trajectory: " + self.trajectory.trajectory_path)
        self.finish_trajectory_publisher.publish()
        self.trajectory.reset()

    def run(self) -> None:
        # TODO could be in cmd with a spin
        while not rospy.is_shutdown():
            if self.trajectory.max_time > 0:
                trajectory_class.send_trajectory()

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("soccer_trajectories")
    trajectory_class = TrajectoryManagerRos()
    try:
        trajectory_class.run()
    except ROSException as ex:
        print(ex)
