#!/usr/bin/env python3

import os

import numpy as np

np.set_printoptions(precision=3)

from soccer_msgs.msg import RobotState

if "ROS_NAMESPACE" not in os.environ:
    os.environ["ROS_NAMESPACE"] = "/robot1"

import math

import rospy
import tf2_ros
from detector import Detector
from geometry_msgs.msg import Pose2D, TransformStamped
from sensor_msgs.msg import JointState

from soccer_common.transformation import Transformation
from soccer_msgs.msg import BoundingBox, BoundingBoxes


class DetectorBall(Detector):
    def __init__(self):
        super().__init__()
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.jointStatesCallback, queue_size=1)
        self.bounding_boxes_sub = rospy.Subscriber("object_bounding_boxes", BoundingBoxes, self.ballDetectorCallback, queue_size=1)
        self.ball_pixel_publisher = rospy.Publisher("ball_pixel", Pose2D, queue_size=1)
        self.head_motor_1_angle = 0
        self.last_ball_pose = None
        self.last_ball_pose_counter = 0

    def jointStatesCallback(self, msg: JointState):
        if len(msg.name) != 0:
            index = msg.name.index("head_motor_1")
            self.head_motor_1_angle = msg.position[index]

    def ballDetectorCallback(self, msg: BoundingBoxes):
        if self.robot_state.status not in [RobotState.STATUS_LOCALIZING, RobotState.STATUS_READY]:
            return

        if not self.camera.ready:
            return

        self.camera.reset_position(timestamp=msg.header.stamp, from_world_frame=True)

        # Ball
        max_detection_size = 0
        final_camera_to_ball: Transformation = None
        final_ball_pixel = None
        candidate_ball_counter = 1
        for box in msg.bounding_boxes:
            if box.Class == "ball":
                # Exclude weirdly shaped balls
                ratio = (box.ymax - box.ymin) / (box.xmax - box.xmin)
                if ratio > 2 or ratio < 0.5:
                    rospy.logwarn_throttle(1, f"Excluding weirdly shaped ball {box.ymax - box.ymin} x {box.xmax - box.xmin}")
                    continue

                boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
                ball_pose = self.camera.calculateBallFromBoundingBoxes(0.07, boundingBoxes)

                # Ignore balls outside of the field
                camera_to_ball = np.linalg.inv(self.camera.pose) @ ball_pose
                detection_size = (box.ymax - box.ymin) * (box.xmax - box.xmin)

                rospy.loginfo_throttle(
                    5,
                    f"Candidate Ball Position { candidate_ball_counter }: { ball_pose.get_position()[0] } { ball_pose.get_position()[1] }, detection_size { detection_size }",
                )
                candidate_ball_counter = candidate_ball_counter + 1

                # Exclude balls outside the field + extra space in the net
                if abs(ball_pose.get_position()[0]) > 5.2 or abs(ball_pose.get_position()[1]) > 3.5:
                    continue

                # Exclude balls that are too far from the previous location
                if self.last_ball_pose is not None:
                    if np.linalg.norm(ball_pose.get_position()[0:2]) < 0.1:  # In the start position
                        pass
                    elif np.linalg.norm(ball_pose.get_position()[0:2] - self.last_ball_pose.get_position()[0:2]) > 3:  # meters from previous position
                        rospy.logwarn_throttle(
                            0.5,
                            f"Detected a ball too far away ({ self.last_ball_pose_counter }), Last Location {self.last_ball_pose.get_position()[0:2]} Detected Location {ball_pose.get_position()[0:2] }",
                        )
                        self.last_ball_pose_counter = self.last_ball_pose_counter + 1
                        if self.last_ball_pose_counter > 5:  # Counter to prevent being stuck when the ball is in a different location
                            self.last_ball_pose_counter = 0
                            self.last_ball_pose = None
                        continue

                # Get the largest detection
                if detection_size > max_detection_size:
                    final_camera_to_ball = camera_to_ball
                    final_ball_pixel = Pose2D()
                    final_ball_pixel.x = (box.xmax - box.xmin) * 0.5 + box.xmin
                    final_ball_pixel.y = (box.ymax - box.ymin) * 0.5 + box.ymin
                    self.last_ball_pose = ball_pose
                    self.last_ball_pose_counter = 0
                    max_detection_size = detection_size
                    pass

        if final_camera_to_ball is not None:
            rospy.loginfo_throttle(
                1,
                f"\u001b[1m\u001b[34mBall detected [{self.last_ball_pose.get_position()[0]:.3f}, {self.last_ball_pose.get_position()[1]:.3f}] \u001b[0m",
            )
            br = tf2_ros.TransformBroadcaster()
            ball_pose = TransformStamped()
            ball_pose.header.frame_id = self.robot_name + "/camera"
            ball_pose.child_frame_id = self.robot_name + "/ball"
            ball_pose.header.stamp = msg.header.stamp
            ball_pose.header.seq = msg.header.seq
            ball_pose.transform.translation.x = final_camera_to_ball.get_position()[0]
            ball_pose.transform.translation.y = final_camera_to_ball.get_position()[1]
            ball_pose.transform.translation.z = final_camera_to_ball.get_position()[2]
            ball_pose.transform.rotation.x = final_camera_to_ball.get_orientation()[0]
            ball_pose.transform.rotation.y = final_camera_to_ball.get_orientation()[1]
            ball_pose.transform.rotation.z = final_camera_to_ball.get_orientation()[2]
            ball_pose.transform.rotation.w = final_camera_to_ball.get_orientation()[3]
            br.sendTransform(ball_pose)

            self.ball_pixel_publisher.publish(final_ball_pixel)

        # Robots
        detected_robots = 0
        for box in msg.bounding_boxes:
            if box.Class == "robot":
                if self.head_motor_1_angle > 0.6:
                    continue  # probably looking at own hands

                x_avg = (box.xmin + box.xmax) / 2.0
                y_avg = (box.ymax + box.ymin) / 2.0
                y_close = box.ymax

                robot_length = abs(box.xmax - box.xmin)
                robot_width = abs(box.ymax - box.ymin)
                if robot_length <= 0 or robot_width <= 0:
                    continue

                length_to_width_ratio_in_full_view = 68 / 22.0
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

                distance = ((floor_center_x - camera_pose.get_position()[0]) ** 2 + (floor_center_y - camera_pose.get_position()[1]) ** 2) ** 0.5
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
