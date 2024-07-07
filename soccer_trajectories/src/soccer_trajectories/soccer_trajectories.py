#!/usr/bin/env python3
import copy
import csv
import os
from typing import Optional

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import rospy
from rospy import ROSException
from scipy.interpolate import interp1d
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty

from soccer_msgs.msg import FixedTrajectoryCommand, RobotState


class Trajectory:
    """Interpolates a CSV trajectory for multiple joints."""

    RATE = 100

    def __init__(self, trajectory_path: str, mirror=False):
        """Initialize a Trajectory from a CSV file at trajectory_path.
        if it's getup trajectory append the desired final pose so the robot is ready for next action
        expects rectangular shape for csv table"""
        self.terminate = False
        self.mirror = mirror
        self.splines = {}
        self.step_map = {}
        self.time_to_last_pose = 2  # seconds
        self.trajectory_path = trajectory_path

        # last_joint_state = JointState()
        # try:
        #     last_joint_state = rospy.wait_for_message("joint_states", JointState, timeout=2)
        # except (ROSException, AttributeError) as ex:
        #     rospy.logerr(ex)
        # except ValueError as ex:
        #     print(ex)

        with open(trajectory_path) as f:
            csv_traj = csv.reader(f)
            for row in csv_traj:
                joint_name = row[0]
                if joint_name == "comment":
                    continue
                if joint_name == "time":
                    self.times = list(map(float, row[1:]))
                    self.times = [0] + self.times + [self.times[-1] + self.time_to_last_pose]
                    self.max_time = self.times[-1]
                else:
                    joint_values = list(map(float, row[1:]))

                    # last_pose_value = float(rospy.get_param(f"motor_mapping/{joint_name}/initial_state"))
                    last_pose_value = 0.0
                    # if joint_name in last_joint_state.name:
                    #     last_pose_value = last_joint_state.position[last_joint_state.name.index(joint_name)]

                    joint_values = [last_pose_value] + joint_values + [last_pose_value]
                    self.splines[joint_name] = interp1d(self.times, joint_values)

    def get_setpoint(self, timestamp):
        """Get the position of each joint at timestamp.
        If timestamp < 0 or timestamp > self.total_time this will throw a ValueError.
        """
        return {joint: spline(timestamp) for joint, spline in self.splines.items()}

    def joints(self):
        """Returns a list of joints in this trajectory."""
        return self.splines.keys()

    def run(self, real_time=True):
        pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=2)
        rate = rospy.Rate(Trajectory.RATE)
        t = 0
        while not rospy.is_shutdown() and t <= self.max_time + 0.01 and not self.terminate:
            js = JointState()
            js.header.stamp = rospy.Time.now()
            for joint, setpoint in self.get_setpoint(round(t * 100) / 100).items():

                if self.mirror:
                    if "left" in joint:
                        joint = joint.replace("left", "right")
                    elif "right" in joint:
                        joint = joint.replace("right", "left")
                js.name.append(joint)
                js.position.append(float(setpoint))

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


class TrajectoryManager:
    def __init__(self):
        use_sim_time_prefix = "_sim" if rospy.get_param("use_sim_time", "false") == "true" else ""
        self.trajectory_path = (
            rospy.get_param_cached(
                "~trajectory_path", os.path.join(os.path.dirname(__file__), "../../trajectories/") + rospy.get_param_cached("robot_model", "bez1")
            )
            + use_sim_time_prefix
        )
        self.trajectory: Optional[Trajectory] = None
        self.command_subscriber = rospy.Subscriber("command", FixedTrajectoryCommand, self.command_callback, queue_size=1)
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


if __name__ == "__main__":
    rospy.init_node("soccer_trajectories")
    trajectory_class = TrajectoryManager()

    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        if trajectory_class.trajectory is not None:
            rospy.loginfo("Running Trajectory: " + trajectory_class.trajectory.trajectory_path + f" {trajectory_class.trajectory.mirror}")
            trajectory_class.trajectory.run()
            rospy.loginfo("Finished Trajectory: " + trajectory_class.trajectory.trajectory_path)
            trajectory_class.finish_trajectory_publisher.publish()
            trajectory_class.trajectory = None

        r.sleep()
