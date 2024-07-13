from os.path import expanduser

import yaml

from soccer_common import PID, Transformation


class Stabilize:
    """
    Manages PID loops for pitch, roll while standing and walking.
    """

    def __init__(
        self,  # TODO should be map read from yaml
        sim: str = "_sim",
        robot_model: str = "bez1",
    ):
        with open(
            expanduser("~") + f"/catkin_ws/src/soccerbot/soccer_control/soccer_pycontrol/config/{robot_model}/{robot_model}{sim}.yaml", "r"
        ) as file:
            parameters = yaml.safe_load(file)
            file.close()

        #: PID values to adjust the torso's front and back movement while standing, getting ready to walk, and post walk
        self.standing_pitch_pid = PID(
            Kp=parameters["standing_pitch_kp"],
            Kd=parameters["standing_pitch_kd"],
            Ki=parameters["standing_pitch_ki"],
            setpoint=parameters["standing_pitch_setpoint"],
            output_limits=(-1.57, 1.57),  # TODO offsets are wrong can fix when apply to motor? or here
        )
        self.standing_pitch_offset = parameters["standing_pitch_offset"]

        #: PID values to adjust the torso's left and right movement while standing, getting ready to walk, and post walk
        self.standing_roll_pid = PID(
            Kp=parameters["standing_roll_kp"],
            Kd=parameters["standing_roll_kd"],
            Ki=parameters["standing_roll_ki"],
            setpoint=parameters["standing_roll_setpoint"],
            output_limits=(-1.57, 1.57),
        )
        self.standing_roll_offset = parameters["standing_roll_offset"]

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

    def reset_imus(self):
        """
        Reset the walking and standing PID values
        """

        self.walking_pitch_pid.reset()
        self.walking_roll_pid.reset()
        self.standing_pitch_pid.reset()
        self.standing_roll_pid.reset()
