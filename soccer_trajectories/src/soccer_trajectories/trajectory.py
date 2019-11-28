from scipy.interpolate import interp1d
import numpy as np
import csv
import rospy
from std_msgs.msg import Float64


# Doesn't safely handle non-rectangular trajectories.
class Trajectory:
    """Interpolates a CSV trajectory for multiple joints for some timestep.

    Attributes:
        splines - A dictionary of splines that serve as the interpolation for each joint
        timestep - The time distance between two points in the interpolation.
        total_time - The total amount of time this trajectory will take.
    """

    def __init__(self, trajectory_path):
        """Initialize a Trajectory from a CSV file at trajectory_path."""
        self.splines = {}
        self.step_map = {}

        with open(trajectory_path) as f:
            csv_traj = csv.reader(f)
            for row in csv_traj:
                if row[0] == 'time':
                    interpolated_time = map(float, row[1:])
                    self.max_time = interpolated_time[-1]
                else:
                    self.splines[row[0]] = interp1d(interpolated_time, map(float, row[1:]))

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
