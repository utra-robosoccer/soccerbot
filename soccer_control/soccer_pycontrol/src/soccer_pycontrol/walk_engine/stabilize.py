from soccer_common import PID, Transformation


class Stabilize:
    def __init__(
        self,  # TODO should be map read from yaml
        standing_pitch_kp: float = 0.1,
        standing_pitch_kd: float = 0,
        standing_pitch_ki: float = 0.0000,
        standing_pitch_setpoint: float = -0.0,
        standing_pitch_offset: float = 0.0,
        standing_roll_kp: float = 0.1,
        standing_roll_kd: float = 0,
        standing_roll_ki: float = 0.00,
        standing_roll_setpoint: float = -0.0,
        standing_roll_offset: float = 0.0,
        walking_pitch_kp: float = 2.3,
        walking_pitch_kd: float = 1,
        walking_pitch_ki: float = 0.0000,
        walking_pitch_setpoint: float = -0.01,
        walking_pitch_offset: float = 0.0,
        walking_roll_kp: float = 1.5,  # TODO remember to change config
        walking_roll_kd: float = 0.5,
        walking_roll_ki: float = 0.00,
        walking_roll_setpoint: float = -0.0,
        walking_roll_offset: float = 0.0,
    ):
        #: PID values to adjust the torso's front and back movement while standing, getting ready to walk, and post walk
        self.standing_pitch_pid = PID(
            Kp=standing_pitch_kp,  # rospy.get_param("standing_pitch_kp", 0.8),
            Kd=standing_pitch_kd,  # rospy.get_param("standing_pitch_kd", 0.0),
            Ki=standing_pitch_ki,  # rospy.get_param("standing_pitch_ki", 0.0005),
            setpoint=standing_pitch_setpoint,  # rospy.get_param("standing_pitch_setpoint", -0.01),
            output_limits=(-1.57, 1.57),  # TODO offsets are wrong can fix when apply to motor? or here
        )
        self.standing_pitch_offset = standing_pitch_offset  # rospy.get_param("walking_pitch_offset", 0.0)

        #: PID values to adjust the torso's left and right movement while standing, getting ready to walk, and post walk
        self.standing_roll_pid = PID(
            Kp=standing_roll_kp,  # rospy.get_param("standing_roll_kp", 0.15),
            Kd=standing_roll_kd,  # rospy.get_param("standing_roll_kd", 0.0),
            Ki=standing_roll_ki,  # rospy.get_param("standing_roll_ki", 0.001),
            setpoint=standing_roll_setpoint,  # rospy.get_param("standing_roll_setpoint", -0.01),
            output_limits=(-1.57, 1.57),
        )
        self.standing_roll_offset = standing_roll_offset  # rospy.get_param("standing_roll_offset", 0.0)

        #: PID values to adjust the torso's front and back movement while walking
        self.walking_pitch_pid = PID(
            Kp=walking_pitch_kp,  # rospy.get_param("walking_pitch_kp", 0.8),
            Kd=walking_pitch_kd,  # rospy.get_param("walking_pitch_kd", 0.0),
            Ki=walking_pitch_ki,  # rospy.get_param("walking_pitch_ki", 0.0005),
            setpoint=walking_pitch_setpoint,  # rospy.get_param("walking_pitch_setpoint", -0.01),
            output_limits=(-1.57, 1.57),
        )
        self.walking_pitch_offset = walking_pitch_offset  # rospy.get_param("walking_pitch_offset", 0.0)

        #: PID values to adjust the torso's left and right movement while walking
        self.walking_roll_pid = PID(
            Kp=walking_roll_kp,  # rospy.get_param("walking_roll_kp", 0.15),
            Kd=walking_roll_kd,  # rospy.get_param("walking_roll_kd", 0.0),
            Ki=walking_roll_ki,  # rospy.get_param("walking_roll_ki", 0.001),
            setpoint=walking_roll_setpoint,  # rospy.get_param("walking_roll_setpoint", -0.01),
            output_limits=(-1.57, 1.57),
        )
        self.walking_roll_offset = walking_roll_offset  # rospy.get_param("walking_roll_offset", 0.0)

    def reset_imus(self):
        """
        Reset the walking and standing PID values
        """

        self.walking_pitch_pid.reset()
        self.walking_roll_pid.reset()
        self.standing_pitch_pid.reset()
        self.standing_roll_pid.reset()
