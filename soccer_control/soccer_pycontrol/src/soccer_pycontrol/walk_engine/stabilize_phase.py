import queue

import numpy as np
from scipy.signal import butter
from soccer_pycontrol.path.path_robot import PathRobot

from soccer_common import PID, Transformation


class StabilizePhase:
    """
    Experimental for a phase roll pid thingy.
    """

    def __init__(
        self,
        walking_roll_Kp: float = 0,
        walking_roll_Kd: float = 0.0,
        walking_roll_Ki: float = 0.05,
        walking_roll_setpoint: float = 0.0,
    ):

        # All related to the roll feedback
        self.roll_feedback_low_pass_filter_butterworth_filter_params = None
        self.roll_feedback_low_pass_filter_order = 8
        self.roll_feedback_steps_per_second = 0  #
        self.roll_feedback_x = queue.Queue(maxsize=self.roll_feedback_low_pass_filter_order + 1)
        self.roll_feedback_y = queue.Queue(maxsize=self.roll_feedback_low_pass_filter_order + 1)
        self.walking_pid_roll = PID(
            Kp=walking_roll_Kp,  # rospy.get_param("walking_roll_Kp", 0),
            Kd=walking_roll_Kd,  # rospy.get_param("walking_roll_Kd", 0),
            Ki=walking_roll_Ki,  # rospy.get_param("walking_roll_Ki", 0.05),
            setpoint=walking_roll_setpoint,  # rospy.get_param("walking_roll_setpoint", 0.0),
            output_limits=(-0.1, 0.1),
        )

    def get_phase_difference_roll(self, t, imu_pose: Transformation):
        # TODO what does this do?
        if imu_pose is None:
            return

        [_, _, roll] = imu_pose.orientation_euler
        cos_value = np.cos(np.array(2 * np.pi * t * self.roll_feedback_steps_per_second / 2))

        x = roll * cos_value

        # https://en.wikipedia.org/wiki/Infinite_impulse_response
        if self.roll_feedback_x.full():
            self.roll_feedback_x.get()
        self.roll_feedback_x.put(x)

        b, a = self.roll_feedback_low_pass_filter_butterworth_filter_params

        sum = 0
        for p in range(len(b)):
            sum += b[p] * self.roll_feedback_x.queue[len(b) - p - 1]

        for q in range(1, len(a)):
            sum -= a[q] * self.roll_feedback_y.queue[len(a) - q]

        y_n = sum / a[0]

        if self.roll_feedback_y.full():
            self.roll_feedback_y.get()
        self.roll_feedback_y.put(y_n)

        return y_n

    def apply_phase_difference_roll_feedback(self, t, imu_pose: Transformation, robot_path: PathRobot):
        """
        Adds IMU feedback while the robot is moving to the arms

        :param imu_pose: Pose of the torso
        :return: The value for the walking_pid controller
        """

        if imu_pose is None:
            return

        y_n = self.get_phase_difference_roll(t, imu_pose)
        F = self.walking_pid_roll.update(y_n)
        return min(t + F, robot_path.duration())

    def reset_imus(self):
        """
        Reset the walking and standing PID values
        """

        self.walking_pid_roll.reset()

    def reset_roll_feedback_parameters(self, robot_path: PathRobot):
        sampling_frequency = 100
        self.roll_feedback_steps_per_second = robot_path.path_sections[0].linearStepCount() / robot_path.path_sections[0].duration()
        cutoff_frequency = self.roll_feedback_steps_per_second / 4
        normalized_cutoff = cutoff_frequency / (0.5 * sampling_frequency)

        # Design the Butterworth filter
        self.roll_feedback_low_pass_filter_butterworth_filter_params = butter(
            self.roll_feedback_low_pass_filter_order, normalized_cutoff, btype="low", analog=False, output="ba"
        )
        for i in range(self.roll_feedback_low_pass_filter_order + 1):
            self.roll_feedback_x.put(0)
            self.roll_feedback_y.put(0)
