# from typing import Union
#
# import scipy
# from soccer_pycontrol.calibration import adjust_navigation_transform
# from soccer_pycontrol.links import Links
# from soccer_pycontrol.path.path_robot import PathRobot
#
# from soccer_common import Transformation
#
#
# class FootStepPlanner:
#     def __init__(self):
#         # TODO lots of duplicates with inversekinematics need to fix it
#         self.robot_path: Union[PathRobot, None] = None
#         self.useCalibration = useCalibration
#
#         #: Dimensions of the foot collision box #TODO get it from URDF
#         self.foot_box = rospy.get_param("foot_box", [0.09, 0.07, 0.01474])
#
#         # : Transformations from the right foots joint position to the center of the collision box of the foot (
#         # https://docs.google.com/presentation/d/10DKYteySkw8dYXDMqL2Klby-Kq4FlJRnc4XUZyJcKsw/edit#slide=id
#         # .g163c1c67b73_0_0)
#         self.right_foot_joint_center_to_collision_box_center = rospy.get_param(
#             "right_foot_joint_center_to_collision_box_center", [0.00385, 0.00401, -0.00737]
#         )
#         self.foot_center_to_floor = -self.right_foot_joint_center_to_collision_box_center[2] + self.foot_box[2]
#
#         pitch_correction = Transformation([0, 0, 0], euler=[0, rospy.get_param("torso_offset_pitch", 0.0), 0])
#         self.torso_offset = Transformation([rospy.get_param("torso_offset_x", 0), 0, 0]) @ pitch_correction
#         #: Odom pose at start of path, reset everytime a new path is created
#         #: Odom pose, always starts at (0,0) and is the odometry of the robot's movement. All odom paths start from odom pose
#         self.odom_pose_start_path = Transformation()
#         self.odom_pose = Transformation()
#
#         self.current_step_time = 0
#
#     def createPathToGoal(self, endPose: Transformation) -> PathRobot:
#         """
#         Creates a path from the robot's current location to the goal location
#
#         :param endPose: 3D transformation
#         :return: Robot path
#         """
#         # TODO why is this here? should be in an integrator
#         startPose = self.setWalkingTorsoHeight(self.pose)
#         endPose = self.setWalkingTorsoHeight(endPose)
#
#         # Remove the roll and yaw from the designated position
#         [r, p, y] = endPose.orientation_euler
#         endPose.orientation_euler = [r, 0, 0]
#
#         # Add calibration
#         if self.useCalibration:
#             endPoseCalibrated = adjust_navigation_transform(startPose, endPose)
#         else:
#             endPoseCalibrated = endPose
#
#         print(
#             f"\033[92mEnd Pose Calibrated: Position (xyz) [{endPoseCalibrated.position[0]:.3f} {endPoseCalibrated.position[1]:.3f} {endPoseCalibrated.position[2]:.3f}], "
#             f"Orientation (xyzw) [{endPoseCalibrated.quaternion[0]:.3f} {endPoseCalibrated.quaternion[1]:.3f} {endPoseCalibrated.quaternion[2]:.3f} {endPoseCalibrated.quaternion[3]:.3f}]\033[0m"
#         )
#
#         self.robot_path = PathRobot(startPose, endPoseCalibrated, self.foot_center_to_floor)
#
#         # obj.rate = rateControl(1 / obj.robot_path.step_size); -- from findPath
#         self.rate = 1 / self.robot_path.step_precision
#         self.period = self.robot_path.step_precision
#
#         self.current_step_time = 0
#
#         self.odom_pose_start_path = deepcopy(self.odom_pose)
#         return self.robot_path
#
#     def stepPath(self, t):
#         """
#         Updates the configuration for the robot for the next position t based on the current path
#
#         :param t: Timestep relative to the time of the first path, where t=0 is the beginning of the path
#         """
#
#         assert t <= self.robot_path.duration()
#
#         # Get Torso position (Average Time: 0.0007538795471191406)
#         torso_position = self.robot_path.torsoPosition(t) @ self.torso_offset
#
#         # Get foot position at time (Average Time: 0.0004878044128417969)
#         [right_foot_position, left_foot_position] = self.robot_path.footPosition(t)
#
#         # Calcualate the feet position relative from torso (Average Time: 0.000133514404296875)
#         torso_to_left_foot = scipy.linalg.lstsq(torso_position, left_foot_position, lapack_driver="gelsy")[0]
#         torso_to_right_foot = scipy.linalg.lstsq(torso_position, right_foot_position, lapack_driver="gelsy")[0]
#
#         # Inverse kinematics for both feet (Average Time: 0.0015840530395507812)
#         thetas = self.inverseKinematicsRightFoot(torso_to_right_foot)
#         self.configuration[Links.RIGHT_LEG_1 : Links.RIGHT_LEG_6 + 1] = thetas[0:6]
#
#         thetas = self.inverseKinematicsLeftFoot(torso_to_left_foot)
#         self.configuration[Links.LEFT_LEG_1 : Links.LEFT_LEG_6 + 1] = thetas[0:6]
#
#         self.pose = torso_position
