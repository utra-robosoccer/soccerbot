# from soccer_common import Transformation
# from soccer_pycontrol.joints import Joints
#
#
# class PID:
#     def __init__(self):
#         #: PID values to adjust the torso's front and back movement while standing, getting ready to walk, and post walk
#         self.standing_pid = PID(
#             Kp=rospy.get_param("standing_Kp", 0.15),
#             Kd=rospy.get_param("standing_Kd", 0.0),
#             Ki=rospy.get_param("standing_Ki", 0.001),
#             setpoint=rospy.get_param("standing_setpoint", -0.01),
#             output_limits=(-1.57, 1.57),
#         )
#         self.standing_offset = rospy.get_param("standing_offset", 0.0)
#
#         #: PID values to adjust the torso's front and back movement while walking
#         self.walking_pid = PID(
#             Kp=rospy.get_param("walking_Kp", 0.8),
#             Kd=rospy.get_param("walking_Kd", 0.0),
#             Ki=rospy.get_param("walking_Ki", 0.0005),
#             setpoint=rospy.get_param("walking_setpoint", -0.01),
#             output_limits=(-1.57, 1.57),
#         )
#         self.walking_offset = rospy.get_param("walking_offset", 0.0)
#
#         # All related to the roll feedback
#         self.roll_feedback_low_pass_filter_butterworth_filter_params = None
#         self.roll_feedback_low_pass_filter_order = 8
#         self.roll_feedback_steps_per_second = 0  #
#         self.roll_feedback_x = queue.Queue(maxsize=self.roll_feedback_low_pass_filter_order + 1)
#         self.roll_feedback_y = queue.Queue(maxsize=self.roll_feedback_low_pass_filter_order + 1)
#         self.walking_pid_roll = PID(
#             Kp=rospy.get_param("walking_roll_Kp", 0),
#             Kd=rospy.get_param("walking_roll_Kd", 0),
#             Ki=rospy.get_param("walking_roll_Ki", 0.05),
#             setpoint=rospy.get_param("walking_roll_setpoint", 0.0),
#             output_limits=(-0.1, 0.1),
#         )
#
#     def apply_imu_feedback(self, imu_pose: Transformation):
#         """
#         Adds IMU feedback while the robot is moving to the arms
#
#         :param imu_pose: Pose of the torso
#         :return: The value for the walking_pid controller
#         """
#
#         if imu_pose is None:
#             return
#
#         [_, pitch, _] = imu_pose.orientation_euler
#         F = self.walking_pid.update(pitch)
#         self.configuration_offset[Joints.LEFT_LEG_3] = F + self.walking_offset
#         self.configuration_offset[Joints.RIGHT_LEG_3] = F + self.walking_offset
#
#         return F
#
#     def apply_imu_feedback_standing(self, imu_pose: Transformation):
#         """
#         Adds IMU feedback while the robot is standing or getting ready to the arms
#
#         :param imu_pose: Pose of the torso
#         :return: The value for the walking_pid controller
#         """
#
#         if imu_pose is None:
#             return
#         [yaw, pitch, roll] = imu_pose.orientation_euler
#         F = self.standing_pid.update(pitch)
#         self.configuration_offset[Joints.LEFT_LEG_5] = F + self.standing_offset
#         self.configuration_offset[Joints.RIGHT_LEG_5] = F + self.standing_offset
#         return pitch
#
#     def get_phase_difference_roll(self, t, imu_pose: Transformation):
#
#         if imu_pose is None:
#             return
#
#         [_, _, roll] = imu_pose.orientation_euler
#         cos_value = np.cos(np.array(2 * np.pi * t * self.roll_feedback_steps_per_second / 2))
#
#         x = roll * cos_value
#
#         # https://en.wikipedia.org/wiki/Infinite_impulse_response
#         if self.roll_feedback_x.full():
#             self.roll_feedback_x.get()
#         self.roll_feedback_x.put(x)
#
#         b, a = self.roll_feedback_low_pass_filter_butterworth_filter_params
#
#         sum = 0
#         for p in range(len(b)):
#             sum += b[p] * self.roll_feedback_x.queue[len(b) - p - 1]
#
#         for q in range(1, len(a)):
#             sum -= a[q] * self.roll_feedback_y.queue[len(a) - q]
#
#         y_n = sum / a[0]
#
#         if self.roll_feedback_y.full():
#             self.roll_feedback_y.get()
#         self.roll_feedback_y.put(y_n)
#
#         return y_n
#
#     def apply_phase_difference_roll_feedback(self, t, imu_pose: Transformation):
#         """
#         Adds IMU feedback while the robot is moving to the arms
#
#         :param imu_pose: Pose of the torso
#         :return: The value for the walking_pid controller
#         """
#
#         if imu_pose is None:
#             return
#
#         y_n = self.get_phase_difference_roll(t, imu_pose)
#         F = self.walking_pid_roll.update(y_n)
#         return min(t + F, self.robot_path.duration())
#
#     def reset_imus(self):
#         """
#         Reset the walking and standing PID values
#         """
#
#         self.walking_pid.reset()
#         self.walking_pid_roll.reset()
#         self.standing_pid.reset()
#
#     def reset_roll_feedback_parameters(self):
#         sampling_frequency = 100
#         self.roll_feedback_steps_per_second = self.robot_path.path_sections[0].linearStepCount() / self.robot_path.path_sections[0].duration()
#         cutoff_frequency = self.roll_feedback_steps_per_second / 4
#         normalized_cutoff = cutoff_frequency / (0.5 * sampling_frequency)
#
#         # Design the Butterworth filter
#         self.roll_feedback_low_pass_filter_butterworth_filter_params = butter(
#             self.roll_feedback_low_pass_filter_order, normalized_cutoff, btype="low", analog=False, output="ba"
#         )
#         for i in range(self.roll_feedback_low_pass_filter_order + 1):
#             self.roll_feedback_x.put(0)
#             self.roll_feedback_y.put(0)
#         pass
