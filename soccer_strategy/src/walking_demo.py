#!/usr/bin/env python3
from robot_ros import RobotRos
from robot import Robot
import rospy
import numpy as np

KICK_TIMEOUT = 3
GETUPFRONT_TIMEOUT = 7
GETUPBACK_TIMEOUT = 10
goal_list = [np.array([2, 2, 0]), np.array([-2, 2, 0]), np.array([-2, -2, 0]), np.array([2, -2, 0])]
current_stage = 0
rospy.init_node("soccer_strategy")

robot = RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY, robot_name="robot1")

rospy.sleep(1)
r = rospy.Rate(10)
while rospy.get_param("walking_engine_ready") == "false":
    r.sleep()

while not rospy.is_shutdown():
    rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9
    if robot.status == Robot.Status.WALKING:
        # publish a goal robot.goal_position geometry_msgs/Pose2D to /robot_name/goal
        pass

    elif robot.status == Robot.Status.FALLEN_BACK:
        robot.terminate_walking_publisher.publish()
        robot.trajectory_publisher.publish("getupback")
        robot.trajectory_complete = False
        robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
        print("getupback")

    elif robot.status == Robot.Status.FALLEN_FRONT:
        robot.terminate_walking_publisher.publish()
        robot.trajectory_publisher.publish("getupfront")
        robot.trajectory_complete = False
        robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
        print("getupback")

    elif robot.status == Robot.Status.READY:
        robot.set_navigation_position(goal_list[current_stage])
        robot.status = Robot.Status.WALKING
        current_stage = (current_stage + 1) % 4

    elif robot.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
        if robot.trajectory_complete:
            robot.status = Robot.Status.READY
        else:
            pass

    if robot.status != robot.previous_status:
        print(robot.robot_name + " status changes to " + str(robot.status))
        robot.previous_status = robot.status

