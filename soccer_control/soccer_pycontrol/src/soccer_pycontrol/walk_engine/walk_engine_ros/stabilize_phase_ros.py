import rospy
from soccer_pycontrol.walk_engine.stabilize_phase import StabilizePhase


class StabilizePhaseROS(StabilizePhase):
    def __init__(self):
        robot_model = rospy.get_param("robot_model", "bez1")
        if rospy.get_param("/use_sim_time", True):
            sim = "_sim"
        else:
            sim = ""
        super(StabilizePhaseROS, self).__init__(robot_model=robot_model, sim=sim)
