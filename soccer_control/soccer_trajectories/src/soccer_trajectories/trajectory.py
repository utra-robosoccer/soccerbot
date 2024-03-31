#!/usr/bin/env python3
import csv
from typing import List, Tuple

from scipy.interpolate import interp1d
from sensor_msgs.msg import JointState


class Trajectory:
    """
    Interpolates a CSV trajectory for multiple joints.
    """

    def __init__(self, trajectory_path: str, mirror=False):
        """
        Initialize a Trajectory from a CSV file at trajectory_path.
        if it's getup trajectory append the desired final pose so the robot is ready for next action
        expects rectangular shape for csv table
        """

        self.mirror = mirror
        self.splines: dict = {}
        self.time_to_last_pose = 2  # seconds

        self.trajectory_path = trajectory_path
        self.max_time: float = 0

    def read_trajectory(self, init_joint_state: JointState) -> None:
        with open(self.trajectory_path) as f:
            csv_traj = csv.reader(f)

            for row in csv_traj:
                joint_name = row[0]

                if joint_name == "comment":
                    continue

                elif joint_name == "time":
                    times = list(map(float, row[1:]))
                    times = [0] + times  # + [self.times[-1] + self.time_to_last_pose]
                    self.max_time = times[-1]

                else:
                    joint_values = list(map(float, row[1:]))

                    # last_pose_value = float(rospy.get_param(f"motor_mapping/{joint_name}/initial_state", 0))
                    init_pose_value = 0
                    if joint_name in init_joint_state.name:
                        init_pose_value = init_joint_state.position[init_joint_state.name.index(joint_name)]

                    joint_values = [init_pose_value] + joint_values  # + [last_pose_value]
                    self.splines[joint_name] = interp1d(times, joint_values)

    def get_set_point(self, timestamp: float) -> dict:
        """Get the position of each joint at timestamp.
        If timestamp < 0 or timestamp > self.total_time this will throw a ValueError.
        """
        _timestamp = round(timestamp * 100) / 100
        return {joint: spline(_timestamp) for joint, spline in self.splines.items()}

    def create_joint_states(self, timestamp: float) -> Tuple[List[str], List[float]]:

        joints = list(self.get_set_point(timestamp).keys())
        joints = self.joint_mirror(joints)
        angles = list(map(float, self.get_set_point(timestamp).values()))

        return joints, angles

    def joint_mirror(self, joints: List[str]) -> List[str]:
        if self.mirror:
            for i, joint in enumerate(joints):
                if "left" in joint:
                    joints[i] = joint.replace("left", "right")
                elif "right" in joint:
                    joints[i] = joint.replace("right", "left")

        return joints

    def reset(self):
        self.splines = {}
        self.max_time = 0
