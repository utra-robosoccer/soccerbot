from scipy.interpolate import interp1d
import numpy as np
import csv
import rospy
from std_msgs.msg import Float64

class Trajectory:
    """Interpolates a CSV trajectory for multiple joints."""

    def __init__(self, trajectory_path):
        """Initialize a Trajectory from a CSV file at trajectory_path.
        if it's getup trajectory append the desired final pose so the robot is ready for next action
        expects rectangular shape for csv table"""
        self.splines = {}
        self.step_map = {}
        self.time_to_last_pose = 5.0  # seconds

        with open(trajectory_path) as f:
            csv_traj = csv.reader(f)
            for row in csv_traj:
                joint_name = row[0]
                if joint_name == 'comment':
                    continue
                if joint_name == 'time':
                    self.times = map(float, row[1:])
                    self.times = [0] + self.times + [self.times[-1] + self.time_to_last_pose]
                    self.max_time = self.times[-1]
                else:
                    joint_values = map(float, row[1:])
                    param = '/soccer_hardware/motor_mapping/{}/initial_state'.format(joint_name)
                    last_pose_value = float(rospy.get_param(param))
                    joint_values = [last_pose_value] + joint_values + [last_pose_value]
                    self.splines[joint_name] = interp1d(self.times, joint_values)

    def get_setpoint(self, timestamp):
        """Get the position of each joint at timestamp.
        If timestamp < 0 or timestamp > self.total_time this will throw a ValueError.
        """
        return {joint: spline(timestamp) for joint, spline in self.splines.iteritems()}

    def joints(self):
        """Returns a list of joints in this trajectory."""
        return self.splines.keys()

    def publish(self):
        publishers = {
            joint: rospy.Publisher("/{}/command".format(joint), Float64, queue_size=10) for joint in self.joints()}

        rate = rospy.Rate(100)
        t = 0
        while not rospy.is_shutdown() and t < self.max_time:
            for joint, setpoint in self.get_setpoint(t).iteritems():
                publishers[joint].publish(setpoint)
            t = t + 0.01
            rate.sleep()
