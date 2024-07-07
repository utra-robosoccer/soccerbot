import rospy
from soccer_pycontrol.walk_engine.stabilize_phase import StabilizePhase


class StabilizePhaseROS(StabilizePhase):
    def __init__(self):
        super(StabilizePhaseROS, self).__init__(
            walking_roll_kp=rospy.get_param("walking_roll_kp", 0),
            walking_roll_kd=rospy.get_param("walking_roll_kd", 0.0),
            walking_roll_ki=rospy.get_param("walking_roll_ki", 0.05),
            walking_roll_setpoint=rospy.get_param("walking_roll_setpoint", -0.0),
        )
