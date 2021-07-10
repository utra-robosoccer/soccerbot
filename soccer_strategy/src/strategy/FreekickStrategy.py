import math
import numpy as np
import rospy

from soccer_strategy.src.strategy.DummyStrategy import DummyStrategy
from soccer_strategy.src.robot.robot import Robot


class FreekickStrategy(DummyStrategy):
    # preparation if we are the kicking team
    def update_kicking_strategy(self, friendly, ball):
        if not self.check_ball_avaliable(ball):
            return False

        non_kicker = []
        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)
            else:
                non_kicker.append(robot)

        if kicker == None:
            # should not happen
            return

        # todo move non-kicking robots

        if np.linalg.norm(kicker.get_position()[0:2] - ball.get_position()) < 0.2:
            # Stop moving
            kicker.set_navigation_position(kicker.get_position())
            return True
        else:
            kicker.set_navigation_position(np.append(ball.get_position(), 0))
            return False

    # kicker actually kick the ball
    def execute_kicking(self, friendly, ball):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if robot.designated_kicker:
                kicker = self.who_has_the_ball(friendly, ball)

        if kicker is None:
            return

        if np.linalg.norm(kicker.get_position()[0:2] - ball.get_position()) < 0.2:
            kicker.set_navigation_position(kicker.get_position())
            kicker.status = Robot.Status.KICKING
        else:
            kicker.set_navigation_position(np.append(ball.get_position(), 0))

    # preparation if we are not the kicking team
    def update_non_kicking_strategy(self, friendly, ball, teamcolor, is_first_half):
        if not self.check_ball_avaliable(ball):
            return

        for robot in friendly:
            if teamcolor == 0: #blue
                if is_first_half == 1:
                    own_goal = np.array([0, -4.5])
                    angle = 1.57
                else:
                    own_goal = np.array([0, 4.5])
                    angle = -1.57
            elif teamcolor == 1: #red
                if is_first_half == 1:
                    own_goal = np.array([0, 4.5])
                    angle = -1.57
                else:
                    own_goal = np.array([0, -4.5])
                    angle = 1.57

            if robot.role == Robot.Role.LEFT_MIDFIELD:
                nav_pose = ball.get_position() + np.array([0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, angle))
            if robot.role == Robot.Role.RIGHT_MIDFIELD:
                nav_pose = ball.get_position() + np.array([-0.5, 0])
                robot.set_navigation_position(np.append(nav_pose, angle))
            if robot.role == Robot.Role.GOALIE:
                nav_pose = own_goal
                #check if robot is on the goal line
                ball_goal_distance = np.linalg.norm(ball.position - own_goal)
                robot_goal_distance = np.linalg.norm(robot.get_position()[0:2] - own_goal)
                if robot_goal_distance <0.5:
                    if ball_goal_distance <1.5:
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
                    robot.set_navigation_position(np.append(nav_pose, angle))

        #todo make is so that all robot stay within the field boundary