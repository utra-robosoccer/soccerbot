from soccer_common import PID


class Stabilize:
    """
    Manages PID loops for pitch, roll while standing and walking.
    """

    def __init__(self, parameters: dict):

        #: PID values to adjust the torso's front and back movement while walking
        self.walking_pitch_pid = PID(
            Kp=parameters["walking_pitch_kp"],
            Kd=parameters["walking_pitch_kd"],
            Ki=parameters["walking_pitch_ki"],
            setpoint=parameters["walking_pitch_setpoint"],
            output_limits=(-1.57, 1.57),
        )
        self.walking_pitch_offset = parameters["walking_pitch_offset"]

        #: PID values to adjust the torso's left and right movement while walking
        self.walking_roll_pid = PID(
            Kp=parameters["walking_roll_kp"],
            Kd=parameters["walking_roll_kd"],
            Ki=parameters["walking_roll_ki"],
            setpoint=parameters["walking_roll_setpoint"],
            output_limits=(-1.57, 1.57),
        )
        self.walking_roll_offset = parameters["walking_roll_offset"]

    # TODO i feel like this can be better organized
    def reset_imus(self):
        """
        Reset the walking and standing PID values
        """

        self.walking_pitch_pid.reset()
        self.walking_roll_pid.reset()
