#!/usr/bin/env python3
import os
from typing import Optional

from rospy import ROSException
from sensor_msgs.msg import JointState

from soccer_trajectories.trajectory import Trajectory

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rospy
from std_msgs.msg import Empty

from soccer_msgs.msg import FixedTrajectoryCommand, RobotState


class TrajectoryManager:
    def __init__(self):
        use_sim_time_prefix = "_sim" if rospy.get_param("use_sim_time", "false") == "true" else ""
        self.trajectory_path = (
                rospy.get_param_cached(
                    "~trajectory_path",
                    os.path.join(os.path.dirname(__file__), "../../trajectories/") + rospy.get_param_cached(
                        "robot_model", "bez1")
                )
                + use_sim_time_prefix
        )
        self.trajectory: Optional[Trajectory] = None
        self.command_subscriber = rospy.Subscriber("command", FixedTrajectoryCommand, self.command_callback,
                                                   queue_size=1)
        self.robot_state_subscriber = rospy.Subscriber("state", RobotState, self.robot_state_callback, queue_size=1)
        self.finish_trajectory_publisher = rospy.Publisher("action_complete", Empty, queue_size=1)

    def robot_state_callback(self, state: RobotState):
        if state.status in [RobotState.STATUS_PENALIZED]:
            if self.trajectory is not None:
                self.trajectory.terminate = True

    def command_callback(self, command: FixedTrajectoryCommand):
        if self.trajectory is not None:
            return

        path = self.trajectory_path + "/" + command.trajectory_name + ".csv"

        if not os.path.exists(path):
            rospy.logerr(f"Trajectory doesn't exist in path {path}")
            return
        self.trajectory = Trajectory(path, command.mirror)

    def run(self, real_time=True):
        # TODO remove from class
        pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=2)
        rate = rospy.Rate(Trajectory.RATE)
        t = 0
        while not rospy.is_shutdown() and t <= self.trajectory.max_time + 0.01 and not self.trajectory.terminate:
            js = self.trajectory.create_joint_states(t)
            js.header.stamp = rospy.Time.now()
            try:
                pub_all_motor.publish(js)
            except ROSException as ex:
                print(ex)
                exit(0)
            t += 0.01
            if int(t + 0.01) != int(t):
                print(f"Trajectory at t={t}")

            if real_time:
                rate.sleep()


if __name__ == "__main__":
    rospy.init_node("soccer_trajectories")
    trajectory_class = TrajectoryManager()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if trajectory_class.trajectory is not None:
            rospy.loginfo(
                "Running Trajectory: " + trajectory_class.trajectory.trajectory_path + f" {trajectory_class.trajectory.mirror}")
            trajectory_class.run()
            rospy.loginfo("Finished Trajectory: " + trajectory_class.trajectory.trajectory_path)
            trajectory_class.finish_trajectory_publisher.publish()
            trajectory_class.trajectory = None

        r.sleep()
