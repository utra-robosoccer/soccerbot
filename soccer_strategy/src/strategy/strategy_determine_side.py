import math

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from strategy.strategy import Strategy, get_back_up
from team import Team
from soccer_msgs.msg import GameState
from robot import Robot
import numpy as np
from strategy.interfaces.actions import Actions


class StrategyDetermineSide(Strategy):
    def __init__(self):
        super().__init__()
        self.average_goal_post_y = 0
        self.update_frequency = 1
        self.measurements = 0
        self.flip_required = False
        self.tf_listener = tf.TransformListener()

    @get_back_up
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        current_robot = self.get_current_robot(friendly_team)
        current_robot.status = Robot.Status.DETERMINING_SIDE

        try:
            goal_post_pose, goal_pose_orientation = self.tf_listener.lookupTransform(
                "robot" + str(current_robot.robot_id) + "/base_camera",
                "robot" + str(current_robot.robot_id) + "/goal_post",
                rospy.Time(0),
            )
            self.average_goal_post_y = self.average_goal_post_y + goal_post_pose[1]
            if goal_post_pose[1] > 0:
                self.average_goal_post_y = self.average_goal_post_y + 1
            else:
                self.average_goal_post_y = self.average_goal_post_y - 1
            self.measurements = self.measurements + 1
            rospy.loginfo("Goal Post detected " + str(goal_post_pose))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate goal post in TF tree")

        if self.measurements == 1:

            # Determine robot position
            self.measurements = self.measurements + 1
            x = abs(current_robot.position_default[0])
            y = -abs(current_robot.position_default[1])
            theta = abs(current_robot.position_default[2])
            if self.average_goal_post_y > 0:
                self.flip_required = True
                y = -y
                theta = -theta
            rospy.loginfo("Robot Position Determined, Determining Roles, Flip Required " + str(self.flip_required))
            current_robot.reset_initial_position([x, y, theta])
            current_robot.position = [x, y, theta]

            # Determining robot role automatically based on distance to role position and other robots
            if current_robot.role == Robot.Role.UNASSIGNED:
                available_roles = []
                for role in friendly_team.formations["ready"]:
                    available_roles.append(role)

                unassigned_robots = []
                for robot in friendly_team.robots:
                    if robot.status == Robot.Status.DISCONNECTED:
                        continue
                    if robot.role == Robot.Role.UNASSIGNED:
                        unassigned_robots.append(robot)

                print("Available Robots", unassigned_robots)
                print("Available Roles", available_roles)
                print(
                    "Available Roles Positions",
                    [friendly_team.formations["ready"][role][0:2] for role in available_roles],
                )
                while len(unassigned_robots) > 0:
                    closest_index = 0
                    closest_role_index = 0
                    closest_distance = math.inf
                    for i in range(len(unassigned_robots)):
                        for j in range(len(available_roles)):
                            distance = np.linalg.norm(
                                np.array(friendly_team.formations["ready"][available_roles[j]][0:2]) - unassigned_robots[i].position[0:2]
                            )
                            if distance < closest_distance:
                                closest_distance = distance
                                closest_index = i
                                closest_role_index = j

                    print(
                        f"Assigning Robot {closest_index + 1} to {available_roles[closest_role_index].name} with location {friendly_team.formations['ready'][available_roles[closest_role_index]][0:2]}"
                    )
                    if unassigned_robots[closest_index].robot_id == current_robot.robot_id:
                        current_robot.role = available_roles[closest_role_index]
                        print("Completed Assignment")
                        break
                    else:
                        unassigned_robots.pop(closest_index)
                        available_roles.pop(closest_role_index)

        if self.measurements > 1:
            current_robot.status = Robot.Status.READY
            current_robot.time_since_action_completed = rospy.Time.now()
