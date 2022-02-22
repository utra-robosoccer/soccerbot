import abc
import rospy
from robot import Robot


class RobotControlled(Robot):

    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED,
                 status=Robot.Status.DISCONNECTED, position=None):
        super().__init__(robot_id, team, role, status, position)

        self.previous_status = self.status
        self.stop_requested = False

    @abc.abstractmethod
    def update_position(self):
        pass

    @abc.abstractmethod
    def terminate_walk(self):
        pass

    @abc.abstractmethod
    def kick(self):
        pass

    @abc.abstractmethod
    def get_back_up(self, type: str="getupback"):
        pass

    def transition_to_next_status(self):
        if self.status != self.previous_status:
            rospy.loginfo("Robot " + str(self.robot_id) + " status changes to " + str(self.status))
            self.previous_status = self.status

        if self.status == Robot.Status.DISCONNECTED:
            self.update_position()

        elif self.status == Robot.Status.READY:
            if self.stop_requested:
                self.status = Robot.Status.STOPPED

        elif self.status == Robot.Status.WALKING:
            if self.stop_requested:
                self.terminate_walk()
                self.status = Robot.Status.STOPPED

        elif self.status == Robot.Status.KICKING:
            self.kick()
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS

        elif self.status == Robot.Status.FALLEN_BACK:
            self.get_back_up("getupback")
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS

        elif self.status == Robot.Status.FALLEN_FRONT:
            self.get_back_up("getupfront")
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS

        elif self.status == Robot.Status.FALLEN_SIDE:
            self.get_back_up("getupside")
            self.status = Robot.Status.TRAJECTORY_IN_PROGRESS

        elif self.status == Robot.Status.TRAJECTORY_IN_PROGRESS:
            rospy.loginfo_throttle(20, "Robot " + str(self.robot_id) + " trajectory in progress")

        elif self.status == Robot.Status.STOPPED:
            pass
        elif self.status == Robot.Status.READY:
            pass

        else:
            rospy.logerr_throttle(20, "Robot " + str(self.robot_id) + " is in invalid status " + str(self.status))