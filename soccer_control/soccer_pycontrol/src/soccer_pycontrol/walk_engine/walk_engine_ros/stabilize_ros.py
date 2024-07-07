import rospy
from soccer_pycontrol.walk_engine.stabilize import Stabilize


class StabilizeROS(Stabilize):
    def __init__(self):
        super(StabilizeROS, self).__init__(
            standing_pitch_kp=rospy.get_param("standing_pitch_kp", 0.15),
            standing_pitch_kd=rospy.get_param("standing_pitch_kd", 0.0),
            standing_pitch_ki=rospy.get_param("standing_pitch_ki", 0.001),
            standing_pitch_setpoint=rospy.get_param("standing_pitch_setpoint", -0.01),
            standing_pitch_offset=rospy.get_param("walking_pitch_offset", 0.0),
            standing_roll_kp=rospy.get_param("standing_roll_kp", 0.1),
            standing_roll_kd=rospy.get_param("standing_roll_kd", 0.0),
            standing_roll_ki=rospy.get_param("standing_roll_ki", 0.001),
            standing_roll_setpoint=rospy.get_param("standing_roll_setpoint", -0.0),
            standing_roll_offset=rospy.get_param("standing_roll_offset", 0.0),
            walking_pitch_kp=rospy.get_param("walking_pitch_kp", 0.8),
            walking_pitch_kd=rospy.get_param("walking_pitch_kd", 0.0),
            walking_pitch_ki=rospy.get_param("walking_pitch_ki", 0.0005),
            walking_pitch_setpoint=rospy.get_param("walking_pitch_setpoint", -0.01),
            walking_pitch_offset=rospy.get_param("walking_pitch_offset", 0.0),
            walking_roll_kp=rospy.get_param("walking_roll_kp", 1.5),  # TODO remember to change config
            walking_roll_kd=rospy.get_param("walking_roll_kd", 0.5),
            walking_roll_ki=rospy.get_param("walking_roll_ki", 0.00),
            walking_roll_setpoint=rospy.get_param("walking_roll_setpoint", -0.0),
            walking_roll_offset=rospy.get_param("walking_roll_offset", 0.0),
        )
