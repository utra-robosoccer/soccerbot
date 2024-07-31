#!/usr/bin/env python3
# TODO rework
#
# import os
# import time
#
# import numpy as np
#
# np.set_printoptions(precision=3)
#
# from soccer_msgs.msg import RobotState
#
# import rospy
# from geometry_msgs.msg import Pose2D
# from sensor_msgs.msg import JointState
# from soccer_object_localization.detector import Detector
#
# from soccer_common.transformation import Transformation
# from soccer_msgs.msg import BoundingBox, BoundingBoxes
#
#
# # if "ROS_NAMESPACE" not in os.environ:
# #     os.environ["ROS_NAMESPACE"] = "/robot1"
#
# # TODO is this really necessary?
#
# class DetectorObjects(Detector):
#     def __init__(self):
#         super().__init__()
#         # self.bounding_boxes_sub = rospy.Subscriber("object_bounding_boxes", BoundingBoxes, self.objectDetectorCallback,
#         #                                            queue_size=1)
#         # self.ball_pixel_publisher = rospy.Publisher("ball_pixel", Pose2D, queue_size=1)
#
#         self.last_ball_pose = None
#         self.last_ball_pose_counter = 0
#         self.candidate_ball_counter = 0
#         self.max_detection_size = 0
#         self.final_camera_to_ball: Transformation = None
#
#     def detect_ball(self, box: BoundingBox):
#         # TODO needs to be reworked
#         # Exclude weirdly shaped balls
#         ratio = (box.ymax - box.ymin) / (box.xmax - box.xmin)
#         if ratio > 2 or ratio < 0.5:
#             # rospy.logwarn_throttle(1,
#             #                        f"Excluding weirdly shaped ball {box.ymax - box.ymin} x {box.xmax - box.xmin}")
#             return True
#
#         boundingBoxes = [[box.xmin, box.ymin], [box.xmax, box.ymax]]
#         ball_pose = self.camera.calculate_ball_from_bounding_boxes(0.07, boundingBoxes)
#
#         # Ignore balls outside of the field
#         camera_to_ball = np.linalg.inv(self.camera.pose) @ ball_pose
#         detection_size = (box.ymax - box.ymin) * (box.xmax - box.xmin)
#
#         # rospy.loginfo_throttle(
#         #     5,
#         #     f"Candidate Ball Position {candidate_ball_counter}: {ball_pose.position[0]} {ball_pose.position[1]}, detection_size {detection_size}",
#         # )
#         self.candidate_ball_counter = self.candidate_ball_counter + 1
#
#         # Exclude balls outside the field + extra space in the net
#         if abs(ball_pose.position[0]) > 5.2 or abs(ball_pose.position[1]) > 3.5:
#             return True
#
#         # If it is the first ball, we need high confidence
#         if self.last_ball_pose is None:
#             if box.probability < rospy.get_param("~first_ball_detection_confidence_threshold", 0.78):
#                 # rospy.logwarn_throttle(0.5,
#                 #                        f"Ignoring first pose of ball with low confidence threshold {box.probability}")
#                 return True
#
#         # Exclude balls that are too far from the previous location
#         if self.last_ball_pose is not None:
#             if np.linalg.norm(ball_pose.position[0:2]) < 0.1:  # In the start position
#                 pass
#             elif np.linalg.norm(ball_pose.position[0:2] - self.last_ball_pose.position[
#                                                           0:2]) > 3:  # meters from previous position
#                 # rospy.logwarn_throttle(
#                 #     0.5,
#                 #     f"Detected a ball too far away ({self.last_ball_pose_counter}), Last Location {self.last_ball_pose.position[0:2]} Detected Location {ball_pose.position[0:2]}",
#                 # )
#                 self.last_ball_pose_counter = self.last_ball_pose_counter + 1
#                 if self.last_ball_pose_counter > 5:  # Counter to prevent being stuck when the ball is in a different location
#                     self.last_ball_pose_counter = 0
#                     self.last_ball_pose = None
#                 return True
#
#         # Get the largest detection
#         if detection_size > self.max_detection_size:
#             self.final_camera_to_ball = camera_to_ball
#             final_ball_pixel = Pose2D()
#             final_ball_pixel.x = (box.xmax - box.xmin) * 0.5 + box.xmin
#             final_ball_pixel.y = (box.ymax - box.ymin) * 0.5 + box.ymin
#             self.last_ball_pose = ball_pose
#             self.last_ball_pose_counter = 0
#             self.max_detection_size = detection_size
#
#         return False
#
#     def objectDetectorCallback(self, msg: BoundingBoxes):
#         # if self.robot_state.status not in [RobotState.STATUS_LOCALIZING, RobotState.STATUS_READY]:
#         #     return
#
#         # if not self.camera.ready():
#         #     return
#
#         # s = time.time()
#         self.camera.reset_position(timestamp=msg.header.stamp, from_world_frame=True, skip_if_not_found=True)
#         # e = time.time()
#         # if e - s > 1:
#         #     rospy.logerr_throttle(1, f"Resetting camera position took longer than usual ({e - s}) seconds")
#
#         # Ball
#         self.max_detection_size = 0
#         self.final_camera_to_ball: Transformation = None
#         final_ball_pixel = None
#         self.candidate_ball_counter = 1
#         obstacle_counter = 0
#         camera_to_obstacle = None
#         # TODO needs to be reworked also some of this should be in strategy and detector node
#         for box in msg.bounding_boxes:
#             if box.Class == "0":
#                 if self.detect_ball(box):
#                     continue
#             elif box.Class == "2":
#                 if box.probability > 0.78:
#                     if box.obstacle_detected:
#                         pos = [box.xbase, box.ybase]
#
#                         floor_coordinate_robot = self.camera.find_floor_coordinate(pos)
#                         world_to_obstacle = Transformation(position=floor_coordinate_robot)
#                         camera_to_obstacle = np.linalg.inv(self.camera.pose) @ world_to_obstacle
#
#                         # TODO do we really need will add back in later
#                         # self.br.sendTransform(
#                         #     camera_to_obstacle.position,
#                         #     camera_to_obstacle.quaternion,
#                         #     msg.header.stamp,
#                         #     self.robot_name + f"/obstacle_{obstacle_counter}",
#                         #     self.robot_name + "/camera",
#                         # )
#                         # rospy.loginfo(f"Obstacle {obstacle_counter} detected at [{pos}] {floor_coordinate_robot} {camera_to_obstacle.position}")
#                         obstacle_counter += 1
#
#         if self.final_camera_to_ball is not None:
#             pass
#             # self.ball_pixel_publisher.publish(final_ball_pixel)
#             # self.br.sendTransform(
#             #     self.final_camera_to_ball.position,
#             #     self.final_camera_to_ball.quaternion,
#             #     msg.header.stamp,
#             #     self.robot_name + "/ball",
#             #     self.robot_name + "/camera",
#             # )
#             # rospy.loginfo_throttle(
#             #     1,
#             #     f"\u001b[1m\u001b[34mBall detected [{self.last_ball_pose.position[0]:.3f}, {self.last_ball_pose.position[1]:.3f}] \u001b[0m",
#             # )
#         return final_ball_pixel, camera_to_obstacle, obstacle_counter
#
# # if __name__ == "__main__":
# # rospy.init_node("ball_detector")
# # ball_detector = DetectorObjects()
# # rospy.spin()
