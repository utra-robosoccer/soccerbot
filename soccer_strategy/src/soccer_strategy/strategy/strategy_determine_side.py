import json
import math
import os

import numpy as np
import rospy
import tf

from soccer_common import Transformation
from soccer_msgs.msg import GameState
from soccer_strategy.robot import Robot
from soccer_strategy.strategy.strategy import Strategy, get_back_up
from soccer_strategy.team import Team


class StrategyDetermineSide(Strategy):
    """
    Initial strategy to determine the position and side of the robot
    """

    def __init__(self):
        super().__init__()
        self.average_goal_post_y = 0
        self.update_frequency = 1
        self.measurements = 0
        self.flip_required = False
        self.tf_listener = tf.TransformListener()

    @get_back_up
    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        """
        Runs a step in the strategy with the frequency update frequency. First tries to get determine the robot's position
        via the goal post information, and if it is not there then timeout and guess the robot's position. Next
        it determines roles based on proximity, and information from other robots

        :param friendly_team: The friendly Team
        :param opponent_team: The opponent Team
        :param game_state: The GameState from the game controller
        """
        super().step_strategy(friendly_team, opponent_team, game_state)

        current_robot = self.get_current_robot(friendly_team)

        if self.iteration == 1:
            current_robot.status = Robot.Status.DETERMINING_SIDE

        try:
            goal_post_pose, goal_pose_orientation = self.tf_listener.lookupTransform(
                "robot" + str(current_robot.robot_id) + "/base_camera",
                "robot" + str(current_robot.robot_id) + "/goal_post",
                rospy.Time(0),
            )
            if goal_post_pose[1] > 0:
                self.average_goal_post_y = self.average_goal_post_y + 1
            else:
                self.average_goal_post_y = self.average_goal_post_y - 1
            self.measurements = self.measurements + 1
            rospy.loginfo("Goal Post detected " + str(goal_post_pose))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn_throttle(30, "Unable to locate goal post in TF tree")

        determine_side_timeout = 0 if rospy.get_param("skip_determine_side", False) else 10
        if (rospy.Time.now() - self.time_strategy_started) > rospy.Duration(determine_side_timeout):
            rospy.logwarn("Timeout error, cannot determine side, determining side as from default")
            self.measurements = 1
            self.average_goal_post_y = 1

        if self.measurements == 1:
            self.measurements += 1
            self.determine_side(current_robot, game_state)
            self.determine_role(current_robot, friendly_team)

            current_robot.status = Robot.Status.READY
            self.complete = True

    def determine_side(self, current_robot, game_state: GameState):
        """
        Determine robot position (assuming the robot is always on the left side

        :param current_robot: The current robot
        """
        team_id = int(os.getenv("ROBOCUP_TEAM_ID", 16))
        if team_id == 16:
            file = "team_1.json"
        else:
            file = "team_2.json"
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../../config/{file}"

        with open(config_folder_path) as json_file:
            team_info = json.load(json_file)

        # TODO determine start pose based on goal post localization instead
        translation = team_info["players"][str(current_robot.robot_id)]["reentryStartingPose"]["translation"]
        rotation = team_info["players"][str(current_robot.robot_id)]["reentryStartingPose"]["rotation"]

        position = [translation[0], translation[1], rotation[3]]
        if self.average_goal_post_y < 0:  # Goal post seen on the left side
            self.flip_required = True
            position[1] = -position[1]
            position[2] = -position[2]
        current_robot.position = position
        rospy.loginfo(f"Robot Position Determined, Determining Roles, Position: {current_robot.position} Flip Required: {self.flip_required}")
        current_robot.reset_initial_position()

    def determine_role(self, current_robot, friendly_team):
        """
        Determining robot role automatically based on distance to role position and other robots

        :param current_robot: The current robot
        :param friendly_team: The friendly team
        :return:
        """
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

            print("  Available Robots", unassigned_robots)
            print("  Available Roles", available_roles)
            print(
                "  Available Roles Positions",
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
                    f"  Assigning Robot {closest_index + 1} { unassigned_robots[closest_index].position } to {available_roles[closest_role_index].name} with location {friendly_team.formations['ready'][available_roles[closest_role_index]][0:2]}"
                )
                if unassigned_robots[closest_index].robot_id == current_robot.robot_id:
                    current_robot.role = available_roles[closest_role_index]
                    print("  Completed Assignment")
                    break
                else:
                    unassigned_robots.pop(closest_index)
                    available_roles.pop(closest_role_index)
