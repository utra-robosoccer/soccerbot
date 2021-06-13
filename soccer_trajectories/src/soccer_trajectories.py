#!/usr/bin/env python3
import soccer_trajectories
import os
from std_msgs.msg import String, Bool
import csv
from scipy.interpolate import interp1d
import rospy
from sensor_msgs.msg import JointState


class Trajectory:
    """Interpolates a CSV trajectory for multiple joints."""

    def __init__(self, trajectory_path):
        """Initialize a Trajectory from a CSV file at trajectory_path.
        if it's getup trajectory append the desired final pose so the robot is ready for next action
        expects rectangular shape for csv table"""
        self.splines = {}
        self.step_map = {}
        self.time_to_last_pose = 2.0  # seconds
        self.motor_names = ["left_arm_motor_0 [shoulder]", "left_arm_motor_1", "right_arm_motor_0 [shoulder]",
                            "right_arm_motor_1",
                            "right_leg_motor_0", "right_leg_motor_1 [hip]", "right_leg_motor_2", "right_leg_motor_3",
                            "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1 [hip]",
                            "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                            "head_motor_0", "head_motor_1"
                            ]

        self.external_motor_names = \
            ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
             "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
             "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1",
             "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
             "head_motor_0", "head_motor_1"
             ]
        with open(trajectory_path) as f:
            csv_traj = csv.reader(f)
            for row in csv_traj:
                joint_name = row[0]
                if joint_name == 'comment':
                    continue
                if joint_name == 'time':
                    self.times = list(map(float, row[1:]))
                    self.times = [0] + self.times + [self.times[-1] + self.time_to_last_pose]
                    self.max_time = self.times[-1]
                else:
                    joint_values = list(map(float, row[1:]))
                    param = '~motor_mapping/{}/initial_state'.format(joint_name)
                    last_pose_value = float(rospy.get_param(param))
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

    def publish(self):
        pub_all_motor = rospy.Publisher("joint_command", JointState, queue_size=10)
        rate = rospy.Rate(100)
        t = 0
        while not rospy.is_shutdown() and t < self.max_time:
            js = JointState()
            js.name = ["left_arm_motor_0", "left_arm_motor_1", "right_arm_motor_0", "right_arm_motor_1",
                       "right_leg_motor_0", "right_leg_motor_1", "right_leg_motor_2", "right_leg_motor_3",
                       "right_leg_motor_4", "right_leg_motor_5", "left_leg_motor_0", "left_leg_motor_1",
                       "left_leg_motor_2", "left_leg_motor_3", "left_leg_motor_4", "left_leg_motor_5",
                       "head_motor_0", "head_motor_1"
                       ]
            js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
            js.position = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

            js.effort = []

            for joint, setpoint in self.get_setpoint(t).items():
                motor_index = js.name.index(joint)
                js.position[motor_index] = setpoint

            pub_all_motor.publish(js)
            t = t + 0.01
            rate.sleep()


class SoccerTrajectoryClass:
    def __init__(self):
        self.trajectory_path = ""
        self.simulation = True
        self.trajectory_complete = True
        self.command_sub = rospy.Subscriber("command", String, self.run_trajectory, queue_size=1)
        self.complete_sub = rospy.Subscriber("trajectory_complete", Bool, self.trajectory_callback, queue_size=1)
        self.finish_trajectory = rospy.Publisher('trajectory_complete', Bool, queue_size=1)

    def trajectory_callback(self, data):
        self.trajectory_complete = data.data
        pass

    def run_trajectory(self, command):
        if self.simulation:
            path = self.trajectory_path + "/" + "simulation_" + command.data + ".csv"
        else:
            path = self.trajectory_path + "/" + command.data + ".csv"

        if not os.path.exists(path):
            return
        if self.trajectory_complete:
            print("Now publishing: ", command.data)
            self.finish_trajectory.publish(False)
            trajectory = Trajectory(path)
            trajectory.publish()
            print("Finished publishing:", command.data)
            self.finish_trajectory.publish(True)

    def run(self):
        rospy.init_node("soccer_trajectories")
        self.trajectory_path = rospy.get_param("~trajectory_path")
        self.simulation = rospy.get_param("~simulation")
        rospy.spin()


if __name__ == '__main__':
    trajectory_class = SoccerTrajectoryClass()
    trajectory_class.run()
