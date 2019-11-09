from scipy.interpolate import interp1d
import numpy as np
import csv
import rospy

#Doesn't safely handle non-rectangular trajectories.
class Trajectory:
    """Interpolates a CSV trajectory for multiple joints for some timestep.

    Attributes:
        splines - A dictionary of splines that serve as the interpolation for each joint
        timestep - The time distance between two points in the interpolation.
        total_time - The total amount of time this trajectory will take.
    """
    def __init__(self, trajectory_path, timestep=0.001):
        """Initialize a Trajectory from a CSV file at trajectory_path."""
        self.splines = {}
        self.timestep = timestep
        n = 0
        with open(trajectory_path) as f:
            csv_traj = csv.reader(f)
            for row in csv_traj:
                n = len(row) - 1
                x_values = np.linspace(0, n * timestep, num=n)
                self.splines[row[0]] = interp1d(x_values, map(float, row[1:]), bounds_error=False, fill_value="extrapolate", kind="cubic", assume_sorted=True)
        self.total_time = n * timestep

    def get_setpoint(self, timestamp):
        """Get the position of each joint at timestamp.
        If timestamp < 0 or timestamp > self.total_time this will throw a ValueError.
        """
        return {joint: spline(timestamp) for joint, spline in self.splines.iteritems()}

    def joints(self):
        """Returns a list of joints in this trajectory."""
        return self.splines.keys()
