#!/usr/bin/env python3

import os

import numpy as np
from soccer_msgs.msg import RobotState

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import math

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped, PoseStamped
from sensor_msgs.msg import JointState
from soccer_object_detection.msg import BoundingBoxes, BoundingBox
from detector import Detector


class DetectorBall(Detector):

    def __init__(self):
        super().__init__()
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.jointStatesCallback)
        self.bounding_boxes_sub = rospy.Subscriber("object_bounding_boxes", BoundingBoxes, self.ballDetectorCallback)
        self.robot_pose_publisher = rospy.Publisher("detected_robot_pose", PoseStamped, queue_size=1)
        self.head_motor_1_angle = 0

    def jointStatesCallback(self, msg: JointState):
        if len(msg.name) != 0:
            index = msg.name.index('head_motor_1')
            self.head_motor_1_angle =  msg.position[index]

    def ballDetectorCallback(self, msg: BoundingBoxes):
        if self.robot_state.status not in [RobotState.STATUS_LOCALIZING, RobotState.STATUS_READY]:
            return

        if not self.camera.ready:
            return

        self.camera.reset_position(timestamp=msg.header.stamp, from_world_frame=True)

        # Ball
        distance_to_robot = math.inf
        final_camera_to_ball = None
        candidate_ball = 1
        for box in msg.bounding_boxes:
            if box.Class == "ball":
                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                ball_pose = self.camera.calculateBallFromBoundingBoxes(0.07, boundingBoxes)

                # Ignore balls outside of the field
                camera_to_ball = np.linalg.inv(self.camera.pose) @ ball_pose
                distance = np.linalg.norm(camera_to_ball[0:2])

                rospy.loginfo_throttle(5, f"Candidate Ball Position { candidate_ball }: { ball_pose.get_position()[0] } { ball_pose.get_position()[1] }, distance { distance }")
                candidate_ball = candidate_ball + 1

                # Exclude balls outside the field
                if abs(ball_pose.get_position()[0]) > 4.5 or abs(ball_pose.get_position()[1]) > 3:
                    continue

                # Get the closest ball to the player
                if distance < distance_to_robot:
                    final_camera_to_ball = camera_to_ball
                    distance_to_robot = distance
                    pass


        if final_camera_to_ball is not None:
            br = tf2_ros.TransformBroadcaster()
            ball_pose = TransformStamped()
            ball_pose.header.frame_id = self.robot_name + "/base_camera"
            ball_pose.child_frame_id = self.robot_name + "/ball"
            ball_pose.header.stamp = msg.header.stamp
            ball_pose.header.seq = msg.header.seq
            ball_pose.transform.translation.x = final_camera_to_ball.get_position()[0]
            ball_pose.transform.translation.y = final_camera_to_ball.get_position()[1]
            ball_pose.transform.translation.z = 0
            ball_pose.transform.rotation.x = 0
            ball_pose.transform.rotation.y = 0
            ball_pose.transform.rotation.z = 0
            ball_pose.transform.rotation.w = 1
            br.sendTransform(ball_pose)


        # Robots
        detected_robots = 0
        for box in msg.bounding_boxes:
            if box.Class == "robot":
                if self.head_motor_1_angle > 0.6:
                    continue  # probably looking at own hands

                x_avg = (box.xmin + box.xmax) / 2.
                y_avg = (box.ymax + box.ymin) / 2.
                y_close = box.ymax
                
                robot_length = abs(box.xmax - box.xmin)
                robot_width = abs(box.ymax - box.ymin)
                if robot_length <= 0 or robot_width <= 0:
                    continue

                length_to_width_ratio_in_full_view = 68 / 22.
                foot_ratio_of_length = 0.2
                foot_pixel = box.ymax

                robot_standing_up = True
                if robot_standing_up:
                    # Side obstructed
                    if box.xmax == 640:
                        # right side obstructed
                        robot_width = robot_length / length_to_width_ratio_in_full_view
                        box.xmax = box.xmin + robot_length
                    elif box.xmin == 0:
                        # left side obstructed
                        robot_width = robot_length / length_to_width_ratio_in_full_view
                        box.xmin = box.xmax - robot_length
                        # If entire robot in field of view
                    if box.ymax == 480 and box.ymin == 0:
                        # Top and bottom both obstructed
                        # TODO dont know what to do at this moment, assume 50 50 top bottom
                        desired_robot_length = length_to_width_ratio_in_full_view * robot_width
                        additional_robot_length = desired_robot_length - robot_length
                        box.ymax = box.ymax + additional_robot_length / 2
                        box.ymin = box.ymin - additional_robot_length / 2
                    elif box.ymax == 480:
                        # bottom obstructed
                        robot_length = length_to_width_ratio_in_full_view * robot_width
                        box.ymax = box.ymin + robot_length
                    elif box.ymin == 0:
                        # top obstructed
                        robot_length = length_to_width_ratio_in_full_view * robot_width
                        box.ymin = box.ymax - robot_length

                    foot_pixel = box.ymax - robot_length * foot_ratio_of_length

                [floor_center_x, floor_center_y, _] = self.camera.findFloorCoordinate([x_avg, foot_pixel])
                [floor_close_x, floor_close_y, _] = self.camera.findFloorCoordinate([x_avg, y_close])

                camera_pose = self.camera.pose

                distance = ((floor_center_x - camera_pose.get_position()[0]) ** 2 + (
                             floor_center_y - camera_pose.get_position()[1]) ** 2) ** 0.5
                theta = math.atan2(distance, camera_pose.get_position()[2])
                ratio = math.tan(theta) ** 2
                ratio2 = 1 / (1 + ratio)
                if 1 < ratio2 < 0:
                    continue

                floor_x = floor_close_x * (1 - ratio2) + floor_center_x * ratio2
                floor_y = floor_close_y * (1 - ratio2) + floor_center_y * ratio2

                if floor_x > 0.0:
                    br = tf2_ros.TransformBroadcaster()
                    robot_pose = TransformStamped()
                    robot_pose.header.frame_id = self.robot_name + "/base_camera"
                    robot_pose.child_frame_id = self.robot_name + "/detected_robot_" + str(detected_robots)
                    robot_pose.header.stamp = msg.header.stamp
                    robot_pose.header.seq = msg.header.seq
                    robot_pose.transform.translation.x = floor_x
                    robot_pose.transform.translation.y = floor_y
                    robot_pose.transform.translation.z = 0
                    robot_pose.transform.rotation.x = 0
                    robot_pose.transform.rotation.y = 0
                    robot_pose.transform.rotation.z = 0
                    robot_pose.transform.rotation.w = 1
                    br.sendTransform(robot_pose)
                    detected_robots += 1

if __name__ == "__main__":
    rospy.init_node("ball_detector")
    ball_detector = DetectorBall()
    rospy.spin()