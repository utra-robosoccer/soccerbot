#!/usr/bin/env python3
import copy
import csv
import os

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
        self.motor_names = [
            "left_arm_motor_0 [shoulder]",
            "left_arm_motor_1",
            "right_arm_motor_0 [shoulder]",
            "right_arm_motor_1",
            "right_leg_motor_0",
            "right_leg_motor_1 [hip]",
            "right_leg_motor_2",
            "right_leg_motor_3",
            "right_leg_motor_4",
            "right_leg_motor_5",
            "left_leg_motor_0",
            "left_leg_motor_1 [hip]",
            "left_leg_motor_2",
            "left_leg_motor_3",
            "left_leg_motor_4",
            "left_leg_motor_5",
            "head_motor_0",
            "head_motor_1",
        ]

        self.external_motor_names = [
            "left_arm_motor_0",
            "left_arm_motor_1",
            "right_arm_motor_0",
            "right_arm_motor_1",
            "right_leg_motor_0",
            "right_leg_motor_1",
            "right_leg_motor_2",
            "right_leg_motor_3",
            "right_leg_motor_4",
            "right_leg_motor_5",
            "left_leg_motor_0",
            "left_leg_motor_1",
            "left_leg_motor_2",
            "left_leg_motor_3",
            "left_leg_motor_4",
            "left_leg_motor_5",
            "head_motor_0",
            "head_motor_1",
        ]
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

                    last_pose_value = float(rospy.get_param(f"motor_mapping/{joint_name}/initial_state"))

                    # last_pose_value = 0.0
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
        pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=10)
        rate = rospy.Rate(Trajectory.RATE)
        t = 0
        while not rospy.is_shutdown() and t < self.max_time and not self.terminate:
            js = JointState()
            js.name = [
                "left_arm_motor_0",
                "left_arm_motor_1",
                "right_arm_motor_0",
                "right_arm_motor_1",
                "right_leg_motor_0",
                "right_leg_motor_1",
                "right_leg_motor_2",
                "right_leg_motor_3",
                "right_leg_motor_4",
                "right_leg_motor_5",
                "left_leg_motor_0",
                "left_leg_motor_1",
                "left_leg_motor_2",
                "left_leg_motor_3",
                "left_leg_motor_4",
                "left_leg_motor_5",
                "head_motor_0",
                "head_motor_1",
            ]
            js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
            js.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

            js.effort = []

            for joint, setpoint in self.get_setpoint(t).items():
                motor_index = js.name.index(joint)
                js.position[motor_index] = float(setpoint)

            if self.mirror:
                position_mirrored = copy.deepcopy(js.position)
                position_mirrored[0:2] = js.position[2:4]
                position_mirrored[2:4] = js.position[0:2]
                position_mirrored[4:10] = js.position[10:16]
                position_mirrored[10:16] = js.position[4:10]
                js.position = position_mirrored

            try:
                pub_all_motor.publish(js)
            except ROSException as ex:
                print(ex)
                exit(0)
            t = t + 0.01
            if real_time:
                rate.sleep()


class SoccerTrajectoryClass:
    def __init__(self):
        self.trajectory_path = rospy.get_param("~trajectory_path", os.path.join(os.path.dirname(__file__), "../../trajectories/bez1"))
        self.trajectory_complete = True
        self.trajectory = None
        self.command_sub = rospy.Subscriber("command", FixedTrajectoryCommand, self.run_trajectory, queue_size=1)
        self.robot_state_sub = rospy.Subscriber("state", RobotState, self.robot_state_callback, queue_size=1)
        self.finish_trajectory = rospy.Publisher("action_complete", Empty, queue_size=1)

    def robot_state_callback(self, state: RobotState):
        if state.status in [RobotState.STATUS_PENALIZED]:
            if self.trajectory is not None:
                self.trajectory.terminate = True

    def run_trajectory(self, command: FixedTrajectoryCommand, real_time=True):
        if not self.trajectory_complete:
            return
        self.trajectory_complete = False

        path = self.trajectory_path + "/" + command.trajectory_name + ".csv"

        if not os.path.exists(path):
            rospy.logerr(f"Trajectory doesn't exist in path {path}")
            self.trajectory_complete = True
            return

        rospy.loginfo("Running Trajectory: " + command.trajectory_name)
        self.trajectory = Trajectory(path, command.mirror)
        self.trajectory.run(real_time=real_time)
        rospy.loginfo("Finished Trajectory: " + command.trajectory_name)
        self.finish_trajectory.publish()
        self.trajectory_complete = True
        return True


if __name__ == "__main__":
    rospy.init_node("soccer_trajectories")
    trajectory_class = SoccerTrajectoryClass()
    rospy.spin()
