#!/usr/bin/env python3
import rospy
import soccer_trajectories
import os
from std_msgs.msg import String, Bool


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
            trajectory = soccer_trajectories.Trajectory(path)
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
