import numpy as np
import rospy
import math

from strategy.strategy import Strategy
from robot import Robot


class StrategyFreekick(Strategy):
    # preparation if we are the kicking team
    def update_kicking_strategy(self, friendly, ball):
        if not self.check_ball_available(ball):
            return False

        non_kicker = []
        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)
            else:
                non_kicker.append(robot)

        if kicker == None:
            return

        # todo move non-kicking robots

        if np.linalg.norm(kicker.position[0:2] - ball.position) < 0.2:
            # Stop moving
            kicker.set_navigation_position(kicker.position)
            return True
        else:
            kicker.set_navigation_position(np.append(ball.position, 0))
            return False

    # kicker actually kick the ball
    def execute_kicking(self, friendly, ball):
        if not self.check_ball_available(ball):
            return

        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)

        if kicker is None:
            return

        if np.linalg.norm(kicker.position[0:2] - ball.position) < 0.2:
            kicker.set_navigation_position(kicker.position)
            kicker.status = Robot.Status.KICKING
        else:
            kicker.set_navigation_position(np.append(ball.position, 0))

    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, game_properties):
        if not self.check_ball_available(ball):
            return

        for robot in friendly:
            if game_properties.team_color == 0: #blue
                goal_position = config.position_map_goal(
                    config.GOAL_POSITION,
                    game_properties.team_color,
                    game_properties.is_first_half,
                    False
                )

                ball_position = ball.position
                diff = ball_position - goal_position
                diff_unit = diff / np.linalg.norm(diff)
                diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

            if robot.role == Robot.Role.LEFT_MIDFIELD:
                nav_pose = ball.position + np.array([0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, diff_angle))
            if robot.role == Robot.Role.RIGHT_MIDFIELD:
                nav_pose = ball.position + np.array([-0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, diff_angle))
            if robot.role == Robot.Role.GOALIE:
                nav_pose = goal_position
                #check if robot is on the goal line
                ball_goal_distance = np.linalg.norm(ball.position - goal_position)
                robot_goal_distance = np.linalg.norm(robot.position[0:2] - goal_position)
                if robot_goal_distance < 0.5:
                    if ball_goal_distance < 1.5:
                        side_delta = self.ball[0]-robot.position[0]
                        if abs(side_delta) < 0.1:
                            pass
                        elif side_delta > 0:
                            robot.terminate_walking_publisher.publish()
                            robot.trajectory_publisher.publish("goalieright")
                            robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
                            rospy.loginfo(self.robot_name + " goalie jump right")
                        else:
                            robot.terminate_walking_publisher.publish()
                            robot.trajectory_publisher.publish("goalieleft")
                            robot.status = Robot.Status.TRAJECTORY_IN_PROGRESS
                            rospy.loginfo(self.robot_name + " goalie jump left")
                else:
                    robot.set_navigation_position(np.append(nav_pose, diff_angle))

        #todo make is so that all robot stay within the field boundary