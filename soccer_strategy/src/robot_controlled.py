import abc
from robot import Robot
import numpy as np
from soccer_pycontrol import path
from soccer_geometry.transformation import Transformation
import math

class RobotControlled(Robot):

    def __init__(self, robot_id=0, team=Robot.Team.UNKNOWN, role=Robot.Role.UNASSIGNED,
                 status=Robot.Status.DISCONNECTED, position=np.array([0, 0, 0])):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status, position=position)

        self.previous_status = self.status
        self.stop_requested = False

        self.start_position = None
        self.goal_position = None
        self.path = None

        self.max_kick_speed = 2
        self.navigation_goal_localized = False # Whether the goal for navigation is clear
        self.kick_with_right_foot = True

    def set_navigation_position(self, goal_position):
        if self.status == Robot.Status.WALKING:
            if self.goal_position is not None and np.linalg.norm(np.array(self.goal_position[0:2]) - np.array(goal_position[0:2])) < 0.05:
                print("New Goal too close to previous goal: New " + str(self.goal_position) + " Old " + str(goal_position))
                return False
            else:
                print("Updating Goal: New " + str(self.goal_position) + " Old " + str(goal_position))

        self.start_position = self.position
        self.goal_position = goal_position
        self.path = path.Path(
            self.position_to_transformation(self.start_position),
            self.position_to_transformation(self.goal_position)
        )
        self.status = Robot.Status.WALKING
        self.navigation_goal_localized = False
        return True

    def position_to_transformation(self, position):
        transfrom_position = (position[0], position[1], 0.)
        q = Transformation.get_quaternion_from_euler([position[2], 0, 0])
        return Transformation(transfrom_position, q)

    def transformation_to_position(self, transform):
        transform_position = transform.get_position()
        transform_quaternion = transform.get_orientation()
        transform_angle = Transformation.get_euler_from_quaternion(transform_quaternion)
        return np.array([transform_position[0], transform_position[1], transform_angle[0]])

    def can_kick(self, ball, goal_position):
        if ball is None or ball.position is None:
            return False

        # generate destination pose
        ball_position = ball.position[0:2]
        player_position = self.position[0:2]
        player_angle = self.position[2]


        # difference between robot angle and nav goal angle
        robot_ball_vector = ball_position - player_position
        robot_ball_angle = math.atan2(robot_ball_vector[1], robot_ball_vector[0])

        nav_angle_diff = (player_angle - robot_ball_angle)
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.205 and abs(nav_angle_diff) < 0.15:
            print("Player {}: Kick | Player Angle {:.3f}, Robot Ball Angle {:.3f}, Nav_angle Diff {:.3f}, Distance Player Ball {:.3f}".
                format(self.robot_id, player_angle, robot_ball_angle, nav_angle_diff, distance_of_player_to_ball))
            if nav_angle_diff > 0.03:
                self.kick_with_right_foot = True
            else:
                self.kick_with_right_foot = False
            return True
        return False

    @abc.abstractmethod
    def terminate_walk(self):
        pass

    @abc.abstractmethod
    def kick(self):
        pass

    @abc.abstractmethod
    def get_back_up(self, type: str="getupback"):
        pass