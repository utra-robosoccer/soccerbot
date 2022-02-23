import math

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped

from strategy.strategy import Strategy
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


    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        current_robot = self.get_current_robot(friendly_team)
        current_robot.status = Robot.Status.DETERMINING_SIDE

        try:
            goal_post_pose, goal_pose_orientation = self.tf_listener.lookupTransform("robot" + str(current_robot.robot_id) + '/base_camera', "robot" + str(current_robot.robot_id) + '/goal_post', rospy.Time(0))
            self.average_goal_post_y = self.average_goal_post_y + goal_post_pose[1]
            if goal_post_pose[1] > 0:
                self.average_goal_post_y = self.average_goal_post_y + 1
            else:
                self.average_goal_post_y = self.average_goal_post_y - 1
            self.measurements = self.measurements + 1
            print("Goal Post detected " + str(goal_post_pose))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate ball in TF tree")

        if self.measurements == 2:
            print("Determining Robot Position based on measurements")
            self.measurements = self.measurements + 1
            if self.average_goal_post_y > 0:
                self.flip_required = True
                current_robot.position[1] = -current_robot.position_default[1]
                current_robot.position[2] = -current_robot.position_default[1]
            self.robot_initial_pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped,
                                                                queue_size=1)
            current_robot.status = Robot.Status.READY

        if self.measurements > 2:
            print("Robot Position Determined, Determining Roles, Flip Required " + str(self.flip_required))

            # Determining robot role automatically based on distance to role position and other robots
            if current_robot.role == Robot.Role.UNASSIGNED:
                available_roles = []
                for role in friendly_team.formations['initial']:
                    available_roles.append(role)

                unassigned_robots = []
                for robot in friendly_team.robots:
                    if robot.status == Robot.Status.DISCONNECTED:
                        continue
                    if robot.role == Robot.Role.UNASSIGNED:
                        unassigned_robots.append(robot)

                print(unassigned_robots)
                print(available_roles)
                while len(unassigned_robots) > 0:
                    closest_index = 0
                    closest_role_index = 0
                    closest_distance = math.inf
                    for i in range(len(unassigned_robots)):
                        for j in range(len(available_roles)):
                            distance = np.linalg.norm(np.array(friendly_team.formations['initial'][available_roles[j]]) - unassigned_robots[i].position)
                            if distance < closest_distance:
                                closest_distance = distance
                                closest_index = i
                                closest_role_index = j

                    print(closest_index)
                    print(closest_role_index)
                    if unassigned_robots[closest_index].robot_id == current_robot.robot_id:
                        print("Assigning")
                        current_robot.role = available_roles[closest_role_index]
                        break
                    else:
                        unassigned_robots.pop(closest_index)
                        available_roles.pop(closest_role_index)


