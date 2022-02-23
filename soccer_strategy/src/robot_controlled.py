import abc
import rospy
from robot import Robot
import numpy as np
import tf
from soccer_pycontrol import path
from soccer_geometry import transformation
import tf.transformations

class RobotControlled(Robot):

    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED,
                 status=Robot.Status.DISCONNECTED, position=np.array([0, 0, 0])):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status, position=position)

        self.previous_status = self.status
        self.stop_requested = False

        self.start_position = None
        self.goal_position = None
        self.path = None

    def set_navigation_position(self, goal_position):
        self.start_position = self.position

        if self.goal_position is not None and np.linalg.norm(self.goal_position - goal_position) < 0.05:
            print("New Goal too close to previous goal: New " + str(self.goal_position) + " Old" + str(goal_position))
            return

        print("New Goal: " + str(goal_position))
        self.goal_position = goal_position
        self.path = path.Path(
            self.position_to_transformation(self.start_position),
            self.position_to_transformation(self.goal_position)
        )
        self.status = Robot.Status.WALKING

    def position_to_transformation(self, position):
        transfrom_position = (position[0], position[1], 0.)
        q = tf.transformations.quaternion_about_axis(position[2], (0, 0, 1))
        transform_quaternion = [q[0], q[1], q[2], q[3]]
        return transformation.Transformation(transfrom_position, transform_quaternion)

    def transformation_to_position(self, transform):
        transform_position = transform.get_position()
        transform_quaternion = transform.get_orientation()
        transfrom_angle = tf.transformations.euler_from_quaternion([
            transform_quaternion[0],
            transform_quaternion[1],
            transform_quaternion[2],
            transform_quaternion[3],
        ])

        return np.array([transform_position[0], transform_position[1], transfrom_angle[2]])

    @abc.abstractmethod
    def terminate_walk(self):
        pass

    @abc.abstractmethod
    def kick(self):
        pass

    @abc.abstractmethod
    def get_back_up(self, type: str="getupback"):
        pass