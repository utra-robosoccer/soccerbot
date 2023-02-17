import math

import numpy as np
import rospy

from soccer_common.utils import wrapToPi
from soccer_strategy.robot import Robot
from soccer_strategy.robot_controlled import RobotControlled


class Utility:
    """
    Reusable functions by multiple strategies
    """

    @staticmethod
    def stop_all_robots(robots: [Robot]):
        """
        Stop all robots from moving
        :param robots: List of robots
        """
        for robot in robots:
            robot.status = Robot.Status.STOPPED

    @staticmethod
    def resume_all_robots(robots: [Robot]):
        """
        Resume all robots
        :param robots: List of robots
        :return:
        """
        for robot in robots:
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY

    @staticmethod
    def navigate_to_scoring_position(robot: RobotControlled, ball_position, goal_position):
        """
        Set a robot to navigate to the most appropriate position for scoring, see the 3 cases below

        There are 3 cases here
          A. If player is close ot the ball and the kicking angle is not bad, and can reach in 10cm, go for the kick directy
          B. If player is far from the ball If the ball is close to the net and not outside the sideline, then go for a 90 degree movement
          C. If the ball is outside the sideline or far from the net, then just go for the center of the net

        :param robot: The robot
        :param ball_position: The position of the ball
        :param goal_position: The center position of the goal net
        :return:
        """

        goal_width = 2.6 - 0.07 * 2  # meters (http://humanoid.robocup.org/wp-content/uploads/RC-HL-2022-Rules-Changes-Marked-3.pdf) - ball_radius

        ball_position = np.array(ball_position)
        player_position = robot.position[0:2]
        player_is_behind_ball = abs(goal_position[0] - ball_position[0]) < abs(goal_position[0] - player_position[0])
        ball_player_diff = ball_position - player_position
        ball_player_distance = np.linalg.norm(ball_player_diff)
        distance_ball_to_net = abs(ball_position[0] - goal_position[0])

        ball_player_diff_unit = ball_player_diff / np.linalg.norm(ball_player_diff)
        tempy = distance_ball_to_net / abs(ball_player_diff_unit[0]) * abs(ball_player_diff_unit[1])
        if player_position[1] > ball_position[1]:
            target_position_y = ball_position[1] - tempy
        else:
            target_position_y = ball_position[1] + tempy
        case_1_dist_ball_to_goal = math.sqrt(tempy**2 + distance_ball_to_net**2)
        case_1_wont_go_out_of_net = abs(target_position_y) < goal_width / 2

        ball_is_very_close_to_net = distance_ball_to_net < 0.4
        player_is_close_to_ball = ball_player_distance < 0.5

        ball_is_close_to_net = distance_ball_to_net < 0.6
        ball_is_in_front_of_net = abs(ball_position[1]) < goal_width / 2
        angle_is_not_bad_for_player = player_is_behind_ball and case_1_dist_ball_to_goal < 0.5

        case_1 = player_is_close_to_ball and ball_is_very_close_to_net and angle_is_not_bad_for_player and case_1_wont_go_out_of_net
        case_2 = ball_is_close_to_net and ball_is_in_front_of_net

        if case_1:
            target_position = np.array([goal_position[0], target_position_y])

            rospy.loginfo(f"Case 1: Target Position {target_position}")
            Utility.navigate_to_position_with_offset(robot, ball_position, target_position, offset=0.168)
        elif case_2:
            target_position = np.array([goal_position[0], ball_position[1]])

            difficult_for_player_to_navigate = abs(ball_position[0]) - 0.1 < abs(player_position[0])

            if difficult_for_player_to_navigate:
                rospy.loginfo(f"Case 2a: Target Position {target_position}")
                Utility.navigate_to_position_with_offset(robot, ball_position, target_position, offset=0.3)
            else:
                rospy.loginfo(f"Case 2b: Target Position {target_position}")
                Utility.navigate_to_position_with_offset(robot, ball_position, target_position)
        else:
            target_position = goal_position

            rospy.loginfo(f"Case 3a: Target Position {target_position}")
            Utility.navigate_to_position_with_offset(robot, ball_position, target_position)

    @staticmethod
    def navigate_to_position_with_offset(robot: RobotControlled, destination_position, target_position, offset=0.173):
        """
        Set the robot to move to the destination position facing the target position with backwards offset

        :param robot: Robot
        :param destination_position: the position of the ball for example
        :param target_position: the position of the net, or where to aim
        :param offset: The distance behind the destination position, on the same line towards the target position, in meters
        :return:
        """

        # generate destination pose
        ball_position = np.array(destination_position)
        player_position = robot.position[0:2]

        diff = target_position - ball_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(diff_unit[1], diff_unit[0])

        destination_position = ball_position - diff_unit * offset

        diff = destination_position - player_position

        # nav bias offset nav goal to be behind the ball
        destination_position_biased = player_position + diff

        # nav goal behind the ball
        destination_position_biased = [destination_position_biased[0], destination_position_biased[1], diff_angle]

        np.set_printoptions(precision=3)
        rospy.loginfo("Player {}: Navigation | Destination position biased {}".format(robot.robot_id, destination_position_biased))
        robot.set_navigation_position(destination_position_biased)

    @staticmethod
    def is_obstacle_blocking(robot: RobotControlled, ball_position):

        obstacle_radius = robot.BODY_WIDTH / 2
        ball_position = np.array(ball_position)
        player_position = robot.position[0:2]
        ball_diff = ball_position - player_position

        for obstacle in robot.observed_obstacles:
            obstacle_position = obstacle.position
            obstacle_diff = obstacle_position - player_position
            if np.linalg.norm(obstacle_diff) > np.linalg.norm(ball_diff):
                continue
            angle = wrapToPi(np.arctan2(obstacle_diff[1], obstacle_diff[0]) - np.arctan2(ball_diff[1], ball_diff[0]))
            opposite_len = np.linalg.norm(obstacle_diff) * math.sin(angle)
            if opposite_len <= obstacle_radius:
                return obstacle.position

        return None

    @staticmethod
    def optimal_position_to_navigate_if_obstacle_blocking_target(robot: RobotControlled, ball_position, obstacle_position):

        player_position = robot.position[0:2]
        ball_position = np.array(ball_position)
        ball_diff = ball_position - player_position
        ball_diff_unit = ball_diff / np.linalg.norm(ball_diff)
        obstacle_diff = obstacle_position - player_position

        ball_diff_rotate_left_90 = np.array([ball_diff[1], -ball_diff[0]])
        ball_diff_rotate_left_90_unit = ball_diff_rotate_left_90 / np.linalg.norm(ball_diff_rotate_left_90)
        ball_diff_rotate_right_90 = np.array([-ball_diff[1], ball_diff[0]])
        ball_diff_rotate_right_90_unit = ball_diff_rotate_right_90 / np.linalg.norm(ball_diff_rotate_right_90)

        angle_obstacle_ball = wrapToPi(np.arctan2(obstacle_diff[1], obstacle_diff[0]) - np.arctan2(ball_diff[1], ball_diff[0]))
        len_to_obstacle_intersect = np.linalg.norm(obstacle_diff) * math.cos(angle_obstacle_ball)

        obstacle_intersect_pt = player_position + ball_diff_unit * len_to_obstacle_intersect
        obstacle_center_to_intersection = obstacle_intersect_pt - obstacle_position

        obstacle_center_to_intersection_unit = obstacle_center_to_intersection / np.linalg.norm(obstacle_center_to_intersection)
        offset_to_optimal_position = 4 * robot.BODY_WIDTH

        if angle_obstacle_ball == 0:
            return obstacle_position + ball_diff_rotate_left_90_unit * offset_to_optimal_position
        else:
            return obstacle_position + obstacle_center_to_intersection_unit * offset_to_optimal_position
