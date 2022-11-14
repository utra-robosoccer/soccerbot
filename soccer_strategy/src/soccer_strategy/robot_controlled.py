import abc
import math

import numpy as np
import rospy

from soccer_common.transformation import Transformation
from soccer_strategy.ball import Ball
from soccer_strategy.robot import Robot
from sensor_msgs.msg import Range
from std_msgs.msg import Header

class RobotControlled(Robot):
    def __init__(
        self,
        robot_id=0,
        team=Robot.Team.UNKNOWN,
        role=Robot.Role.UNASSIGNED,
        status=Robot.Status.DISCONNECTED,
        position=np.array([0, 0, 0]),

    ):
        super().__init__(robot_id=robot_id, team=team, role=role, status=status, position=position)

        self.previous_status = self.status
        self.stop_requested = False

        self.start_position = None
        self.goal_position = None

        self.max_kick_speed = 2
        self.navigation_goal_localized_time = rospy.Time.now()
        self.kick_with_right_foot = True
        self.kicking_range_publisher = rospy.Publisher("strategy/kicking_range", Range, queue_size=1, latch=True)

    def shorten_navigation_position(self, goal_position):
        """
        Limits the length of the navigation, for example a goal given 3 meters will return a goal determined by
        the shorten_navigation_limit parameter, if it is 1 then 1 meter max navigation limit
        :param goal_position: [x, y, theta] of the robot's goal position
        """

        shorten_navigation_limit = rospy.get_param("shorten_navigation_limit", 3)
        distance_xy = np.linalg.norm(self.position[0:2] - goal_position[0:2])
        if distance_xy < shorten_navigation_limit:
            return goal_position

        diff = goal_position[0:2] - self.position[0:2]
        diff_unit = diff / np.linalg.norm(diff)
        diff_unit *= shorten_navigation_limit

        diff_angle = math.atan2(diff[0], diff[1])
        diff_angle *= shorten_navigation_limit

        return np.array([diff_unit[0], diff_unit[1], diff_angle] + self.position)

    def set_navigation_position(self, goal_position):
        """
        Set's the robot's navigation position for the robot to go to
        :param goal_position: goal_position: [x, y, theta] of the robot's goal position
        """

        if self.status == Robot.Status.WALKING:
            if self.goal_position is not None and np.linalg.norm(np.array(self.goal_position[0:2]) - np.array(goal_position[0:2])) < 0.05:
                rospy.logwarn("New Goal too close to previous goal: New " + str(self.goal_position) + " Old " + str(goal_position))
                return False
            else:
                rospy.logwarn("Updating Goal: New " + str(self.goal_position) + " Old " + str(goal_position))

        self.start_position = self.position
        self.goal_position = goal_position
        self.status = Robot.Status.WALKING
        return True

    def can_kick(self, ball: Ball, goal_position):
        """
        Whether the robot can kick the ball based on the robot and ball position

        :param ball: Ball object
        :param goal_position: [x, y, theta] of the robot's goal position
        :return: True if the robot can kick the ball
        """

        if ball is None or ball.position is None:
            return False

        # generate destination pose
        ball_position = ball.position[0:2]
        player_position = self.position[0:2]
        player_angle = self.position[2]

        # difference between robot angle and nav goal angle
        robot_ball_vector = ball_position - player_position
        robot_ball_angle = math.atan2(robot_ball_vector[1], robot_ball_vector[0])

        nav_angle_diff = player_angle - robot_ball_angle
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        # Initialize and create a Range visualizer for kicking angle
        kicking_angle_header = Header()
        kicking_angle_header.stamp = rospy.Time.now()
        kicking_angle_header.frame_id = f"robot{self.robot_id}/torso"
        kicking_angle = Range(header = kicking_angle_header, radiation_type = 1, field_of_view = abs(nav_angle_diff), min_range = 1, max_range = 2, range = 2)
        
        # Publishing range to topic /strategy/kicking_angle

        self.kicking_range_publisher.publish(kicking_angle)

        if distance_of_player_to_ball < rospy.get_param("min_kick_distance", 0.20) and abs(nav_angle_diff) < rospy.get_param("min_kick_angle", 0.4):
            if nav_angle_diff > 0:
                self.kick_with_right_foot = True
            else:
                self.kick_with_right_foot = False
            print(
                "\u001b[1m\u001b[34mPlayer {}: Kick | Player Angle {:.3f}, Robot Ball Angle {:.3f}, Nav_angle Diff {:.3f}, Distance Player Ball {:.3f}, Right Foot {}\u001b[0m".format(
                    self.robot_id,
                    player_angle,
                    robot_ball_angle,
                    nav_angle_diff,
                    distance_of_player_to_ball,
                    self.kick_with_right_foot,
                )
            )
            return True
        return False

    @abc.abstractmethod
    def terminate_walk(self):
        """
        Send a command to stop the robot's walking
        """
        pass

    @abc.abstractmethod
    def kick(self):
        """
        Send a command to kick the ball
        """
        pass

    @abc.abstractmethod
    def get_back_up(self, type: str = "getupback"):
        """
        Send a command get back up, based on the get up type
        """
        pass

    @abc.abstractmethod
    def reset_initial_position(self):
        """
        Set's the robot's position based on the amcl_pose
        :return:
        """
        pass

    def set_kick_velocity(self, kick_velocity):
        """
        TODO deprecate and delete
        """
        pass
