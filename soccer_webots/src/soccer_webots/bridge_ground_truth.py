#!/usr/bin/env python3

import os

import numpy as np
import rospy
import tf
import transforms3d
from controller import Node, Supervisor
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from soccer_common.transformation import Transformation

ROBOTS = ["robot1", "robot2", "robot3", "robot4"]
os.environ["WEBOTS_ROBOT_NAME"] = "BridgeGroundTruth"

supervisor = Supervisor()
supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_REAL_TIME)
transform_broadcaster = None


def reset_robot(pose: Pose, robot: str):
    robotdef = supervisor.getFromDef(robot)
    robotdef.getField("translation").setSFVec3f([pose.position.x, pose.position.y, 0.0772])
    axis, angle = transforms3d.quaternions.quat2axangle([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    robotdef.getField("rotation").setSFRotation(list(np.append(axis, angle)))
    robotdef.resetPhysics()


def reset_ball(pose: Pose):
    ball_def = supervisor.getFromDef("ball")
    ball_def.getField("translation").setSFVec3f([pose.position.x, pose.position.y, 0.0772])
    ball_def.getField("rotation").setSFRotation([0, 0, 1, 0])
    ball_def.resetPhysics()


def publish_ground_truth_messages(c: Clock):
    for name in ROBOTS:
        robot_def = supervisor.getFromDef(name)
        ball_def = supervisor.getFromDef("ball")

        if robot_def is not None:
            # Robot Position Odom
            translation = robot_def.getField("translation").getSFVec3f()
            rotation_angleax = robot_def.getField("rotation")
            quat_scalar_first = transforms3d.quaternions.axangle2quat(rotation_angleax[:3], rotation_angleax[3])
            rotation_quaternion = np.append(quat_scalar_first[1:], quat_scalar_first[0])

            odom = rospy.Publisher("/" + name + "/base_pose_ground_truth", Odometry, queue_size=1)
            odometry = Odometry()
            odometry.header.frame_id = name + "/odom"
            odometry.child_frame_id = name + "/base_footprint"
            odometry.header.stamp = rospy.Time.from_seconds(c.clock.secs)
            odometry.pose.pose.position.x = translation[0]
            odometry.pose.pose.position.y = translation[1]
            odometry.pose.pose.position.z = 0.0
            odometry.pose.pose.orientation.x = rotation_quaternion[0]
            odometry.pose.pose.orientation.y = rotation_quaternion[1]
            odometry.pose.pose.orientation.z = rotation_quaternion[2]
            odometry.pose.pose.orientation.w = rotation_quaternion[3]

            # fmt: off
            odometry.pose.covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
            # fmt: on
            odom.publish(odometry)
            transform_broadcaster.sendTransform(
                (odometry.pose.pose.position.x, odometry.pose.pose.position.y, odometry.pose.pose.position.z),
                (
                    odometry.pose.pose.orientation.x,
                    odometry.pose.pose.orientation.y,
                    odometry.pose.pose.orientation.z,
                    odometry.pose.pose.orientation.w,
                ),
                c.clock,
                name + "/base_footprint_gt",
                "world",
            )

            # Robot Camera Position
            camera_node = supervisor.getFromDef("robot1").getFromProtoDef("CAMERA")
            camera_pos = camera_node.getPosition()
            adj_matrix = Transformation.get_matrix_from_euler([0, np.pi / 2, -np.pi / 2])
            orient_matrix = np.reshape(camera_node.getOrientation(), (3, 3))
            camera_quat = Transformation.get_quaternion_from_rotation_matrix(orient_matrix @ adj_matrix)
            transform_broadcaster.sendTransform(
                (camera_pos[0], camera_pos[1], camera_pos[2]),
                (camera_quat[0], camera_quat[1], camera_quat[2], camera_quat[3]),
                c.clock,
                name + "/camera_gt",
                "world",
            )

            # Ball Position
            if ball_def is not None:
                pos = ball_def.getField("translation").getSFVec3f()
                orient = ball_def.getField("rotation").getSFRotation()
                quat_scalar_first = transforms3d.quaternions.axangle2quat(orient[:3], orient[3])
                quat_scalar_last = np.append(quat_scalar_first[1:], quat_scalar_first[0])
                orient = list(quat_scalar_last)
                transform_broadcaster.sendTransform(
                    (pos[0], pos[1], pos[2]),
                    (orient[0], orient[1], orient[2], orient[3]),
                    c.clock,
                    name + "/ball_gt",
                    "world",
                )


if __name__ == "__main__":

    rospy.init_node("groud_truth_bridge")
    transform_broadcaster = tf.TransformBroadcaster()

    clock_subscriber = rospy.Subscriber("clock", Clock, callback=publish_ground_truth_messages)

    reset_robot_subscribers = []
    for robot in ROBOTS:
        reset_robot_subscribers.append(rospy.Subscriber("/reset_robot", Pose, reset_robot, robot))

    reset_ball_subscriber = rospy.Subscriber("/reset_ball", Pose, reset_ball)
    rospy.spin()
