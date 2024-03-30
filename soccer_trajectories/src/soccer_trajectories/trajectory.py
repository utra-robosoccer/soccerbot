#!/usr/bin/env python3
import csv
import os
from typing import List

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
        self.splines = {}
        self.time_to_last_pose = 2  # seconds

        self.trajectory_path = trajectory_path
        self.max_time = 0
        #
        # TODO add unit testing

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

    def get_setpoint(self, timestamp: float) -> dict:
        """Get the position of each joint at timestamp.
        If timestamp < 0 or timestamp > self.total_time this will throw a ValueError.
        """
        _timestamp = round(timestamp * 100) / 100
        return {joint: spline(_timestamp) for joint, spline in self.splines.items()}

    def create_joint_states(self, timestamp: float) -> JointState:
        js = JointState()
        for joint, setpoint in self.get_setpoint(timestamp).items():
            joint = self.joint_mirror(joint)

            js.name.append(joint)

            js.position.append(float(setpoint))
        return js

    def create_pybullet_states(self, timestamp: float, motor_name: List[str]) -> List[float]:
        states = [0.0] * 18
        for index, name in enumerate(motor_name):
            for joint, setpoint in self.get_setpoint(timestamp).items():

                joint = self.joint_mirror(joint)

                if joint == name:
                    states[index] = float(setpoint)
                    break
        return states

    def joint_mirror(self, joint: str) -> str:
        if self.mirror:
            if "left" in joint:
                joint = joint.replace("left", "right")
            elif "right" in joint:
                joint = joint.replace("right", "left")

        return joint


if __name__ == "__main__":
    traj = Trajectory(os.path.join(os.path.dirname(__file__), "../../trajectories/bez1_sim/getupfront.csv"))
    #
    joint_state = JointState()
    joint_state.name = [
        "right_leg_motor_0",
        "left_leg_motor_0",
        "right_leg_motor_1",
        "left_leg_motor_1",
        "right_leg_motor_2",
        "left_leg_motor_2",
        "right_leg_motor_3",
        "left_leg_motor_3",
        "right_leg_motor_4",
        "left_leg_motor_4",
        "right_leg_motor_5",
        "left_leg_motor_5",
        "right_arm_motor_0",
        "left_arm_motor_0",
        "right_arm_motor_1",
        "left_arm_motor_1",
        "head_motor_0",
        "head_motor_1",
    ]
    joint_state.position = [0.0] * 18

    traj.read_trajectory(joint_state)

    print(traj.create_joint_states(traj.max_time).position)
