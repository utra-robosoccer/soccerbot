from scipy.interpolate import interp1d
import numpy as np
import csv
import rospy
from webots_ros.srv import set_float, set_floatRequest
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
import time


class Trajectory:
    """Interpolates a CSV trajectory for multiple joints."""

    def __init__(self, trajectory_path):
        """Initialize a Trajectory from a CSV file at trajectory_path.
        if it's getup trajectory append the desired final pose so the robot is ready for next action
        expects rectangular shape for csv table"""
        self.splines = {}
        self.step_map = {}
        self.time_to_last_pose = 2.0  # seconds

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
        publishers = {
            joint: rospy.Publisher("{}/command".format(joint), Float64, queue_size=10) for joint in self.joints()}
        # r = rospy.Rate(10)

        r = rospy.Rate(10)
        while not rospy.has_param("competition"):
            r.sleep()
        self.competition = rospy.get_param("competition")
        '''while rospy.has_param("/robot1/controller_name") == False:
            r.sleep()

        print(rospy.has_param("/robot1/controller_name"))
        controllerName = rospy.get_param("/robot1/controller_name")
        publishers_2 = {
            joint: rospy.ServiceProxy("/" + controllerName + "/{}/set_position".format(joint), set_float) for joint in self.joints()}

        print(publishers_2)'''
        pub_all_motor = rospy.Publisher("all_motor", JointState, queue_size=10)

        rate = rospy.Rate(100)
        t = 0
        while not rospy.is_shutdown() and t < self.max_time:
            if self.competition == "False":
                js = JointState()
                js.name = []
                js.header.stamp = rospy.Time.now()  # rospy.Time.from_seconds(self.time)
                js.position = []
                js.effort = []
                for joint, setpoint in self.get_setpoint(t).items():
                    js.name.append(joint)
                    js.position.append(setpoint)

                pub_all_motor.publish(js)
            elif self.competition == "True":
                for joint, setpoint in self.get_setpoint(t).items():
                    publishers[joint].publish(setpoint)
            t = t + 0.01
            rate.sleep()
