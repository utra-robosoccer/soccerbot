import math

import numpy as np
import rospy
from ball import Ball
from robot import Robot
from robot_controlled import RobotControlled
from team import Team


class Actions:
    @staticmethod
    def stop_all_robots(robots: [Robot]):
        for robot in robots:
            robot.status = Robot.Status.STOPPED

    @staticmethod
    def resume_all_robots(robots: [Robot]):
        for robot in robots:
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY

    @staticmethod
    def kick(robot: [Robot], ball: Ball, target_position):
        player_position = robot.position[0:2]
        ball_position = np.array(ball.position)
        player_angle = robot.position[2]
        diff = ball_position - target_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle), math.cos(player_angle - diff_angle))
        distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)

        if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) < 0.15 and robot.path.isFinished(robot.path_time):
            if nav_angle__diff > 0.03:
                # right foot
                robot.kick_with_right_foot = True
            else:
                robot.kick_with_right_foot = False

            delta = target_position - ball_position
            unit = delta / np.linalg.norm(delta)

            robot.status = Robot.Status.KICKING
            robot.set_kick_velocity(unit * robot.max_kick_speed)

    @staticmethod
    def navigation_to_formation(team: Team, formation: str):
        for robot in team.robots:
            Actions.navigation_to_position(robot, team.formations[formation][robot.role])

    @staticmethod
    def navigation_to_position(robot: RobotControlled, position):
        robot.set_navigation_position(position)

    @staticmethod
    def navigate_to_scoring_position(robot, destination_position, goal_position):
        goal_width = 2.6 - 0.07 * 2  # meters (http://humanoid.robocup.org/wp-content/uploads/RC-HL-2022-Rules-Changes-Marked-3.pdf) - ball_radius

        ball_position = np.array(destination_position)
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

        # There are 3 cases here
        # A. If player is close ot the ball and the kicking angle is not bad, and can reach in 10cm, go for the kick directy
        # B. If player is far from the ball If the ball is close to the net and not outside the sideline, then go for a 90 degree movement
        # C. If the ball is outside the sideline or far from the net, then just go for the center of the net
        case_1 = player_is_close_to_ball and ball_is_very_close_to_net and angle_is_not_bad_for_player and case_1_wont_go_out_of_net
        case_2 = ball_is_close_to_net and ball_is_in_front_of_net

        if case_1:
            target_position = np.array([goal_position[0], target_position_y])

            rospy.loginfo(f"Case 1: Target Position {target_position}")
            Actions.navigate_to_position_with_offset(robot, ball_position, target_position, offset=0.168)
        elif case_2:
            target_position = np.array([goal_position[0], ball_position[1]])

            difficult_for_player_to_navigate = abs(ball_position[0]) - 0.1 < abs(player_position[0])

            if difficult_for_player_to_navigate:
                rospy.loginfo(f"Case 2a: Target Position {target_position}")
                Actions.navigate_to_position_with_offset(robot, ball_position, target_position, offset=0.3)
            else:
                rospy.loginfo(f"Case 2b: Target Position {target_position}")
                Actions.navigate_to_position_with_offset(robot, ball_position, target_position)
        else:
            target_position = goal_position

            rospy.loginfo(f"Case 3a: Target Position {target_position}")
            Actions.navigate_to_position_with_offset(robot, ball_position, target_position)

    @staticmethod
    def navigate_to_position_with_offset(robot, destination_position, target_position, offset=0.173):
        # Destination position = the position of the ball for example
        # Target Position = the position of the net, or where to aim

        # generate destination pose
        ball_position = np.array(destination_position)
        player_position = robot.position[0:2]

        diff = ball_position - target_position
        diff_unit = diff / np.linalg.norm(diff)
        diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

        destination_position = ball_position + diff_unit * offset

        diff = destination_position - player_position

        # nav bias offset nav goal to be behind the ball
        destination_position_biased = player_position + diff

        # nav goal behind the ball
        destination_position_biased = [destination_position_biased[0], destination_position_biased[1], diff_angle]

        np.set_printoptions(precision=3)
        rospy.loginfo("Player {}: Navigation | Destination position biased {}".format(robot.robot_id, destination_position_biased))
        Actions.navigation_to_position(robot, destination_position_biased)
