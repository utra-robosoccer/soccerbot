import copy
import json
import math
import os
from typing import Optional

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
            current_robot.localized = False

        if not current_robot.localized:

            footprint_to_goal_post = None
            # Transform of base footprint to goal post
            try:
                goal_post_position, goal_post_orientation = self.tf_listener.lookupTransform(
                    os.environ["ROS_NAMESPACE"].replace("/", "") + "/base_footprint",
                    os.environ["ROS_NAMESPACE"].replace("/", "") + "/goal_post",
                    rospy.Time(0),
                )
                print(f"Detected Goal Post Position {goal_post_position}, Orientation {goal_post_orientation}")
                footprint_to_goal_post = Transformation(position=goal_post_position, quaternion=goal_post_orientation)
                rospy.loginfo(f"Footprint to goal post: {footprint_to_goal_post.position}")

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

                rospy.logwarn_throttle(30, "Unable to get robot to camera pose")

            determine_side_timeout = 0 if rospy.get_param("skip_determine_side", False) else 10
            if (rospy.Time.now() - self.time_strategy_started) > rospy.Duration(determine_side_timeout):
                rospy.logwarn("Timeout error, cannot determine side, determining side as from default")
                self.determine_side_initial(current_robot, game_state)
                self.determine_role(current_robot, friendly_team)
                current_robot.status = Robot.Status.READY
                current_robot.localized = True
            elif footprint_to_goal_post is not None:
                side_determined = self.determine_side(current_robot, footprint_to_goal_post, game_state)
                if not side_determined:
                    return

                self.determine_role(current_robot, friendly_team)
                current_robot.status = Robot.Status.READY
                current_robot.localized = True
        else:
            for robot in friendly_team.robots:
                if robot.status is Robot.Status.DETERMINING_SIDE and not robot.localized:
                    rospy.logwarn_throttle(1, f"robot {robot.robot_id} has not determined position")
                    return
            self.determine_role(current_robot, friendly_team)

            current_robot.status = Robot.Status.READY
            self.complete = True

    def determine_side_initial(self, current_robot, game_state: GameState):
        team_id = int(os.getenv("ROBOCUP_TEAM_ID", 16))
        if team_id == 16:
            file = "team_1.json"
        else:
            file = "team_2.json"
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../../config/{file}"

        with open(config_folder_path) as json_file:
            team_info = json.load(json_file)

        translation = team_info["players"][str(current_robot.robot_id)]["reentryStartingPose"]["translation"]
        rotation = team_info["players"][str(current_robot.robot_id)]["reentryStartingPose"]["rotation"]

        position = np.array([translation[0], translation[1], rotation[3]])
        current_robot.position = position
        rospy.loginfo(f"Robot Position Determined, Determining Roles, Position: {current_robot.position}")
        current_robot.reset_initial_position()

    def determine_side(self, current_robot, footprint_to_goal_post: Transformation, game_state: GameState) -> bool:

        if footprint_to_goal_post.position[0] < 0:
            return False

        team_id = int(os.getenv("ROBOCUP_TEAM_ID", 16))
        if team_id == 16:
            file = "team_1.json"
        else:
            file = "team_2.json"
        file_path = os.path.dirname(os.path.abspath(__file__))
        config_folder_path = f"{file_path}/../../../config/{file}"

        with open(config_folder_path) as json_file:
            team_info = json.load(json_file)

        # Rulebook http://humanoid.robocup.org/wp-content/uploads/RC-HL-2022-Rules-Changes-Marked-3.pdf
        FIELD_LENGTH = 9
        FIELD_WIDTH = abs(
            team_info["players"]["1"]["reentryStartingPose"]["translation"][1]
        )  # m, how far the robot stands from the center in y direction
        GOAL_WIDTH = 2.6

        # Top left, top right, bottom left, bottom right, assuming x is the long for the net
        goal_post_locations = [
            (-FIELD_LENGTH / 2, GOAL_WIDTH / 2),  # Left far
            (FIELD_LENGTH / 2, GOAL_WIDTH / 2),  # Right far
            (-FIELD_LENGTH / 2, -GOAL_WIDTH / 2),  # Left close
            (FIELD_LENGTH / 2, -GOAL_WIDTH / 2),  # Right close
        ]

        # Where the robot would be if the goal post detected is there, adjusted for distance
        potential_robot_locations = []
        for l in goal_post_locations:
            if footprint_to_goal_post.position[0] == 0:
                return False

            # If the robot is facing forward
            y_delta = l[1] + FIELD_WIDTH
            x_delta = y_delta / footprint_to_goal_post.position[0] * footprint_to_goal_post.position[1]
            position = [l[0] + x_delta, l[1] - y_delta]
            if position[0] <= 0 and 0.7 < abs(y_delta / footprint_to_goal_post.position[0]) < 1.2:
                potential_robot_locations.append(position)

            # If the robot is on the other side
            y_delta = FIELD_WIDTH - l[1]
            x_delta = y_delta / footprint_to_goal_post.position[0] * footprint_to_goal_post.position[1]
            position = [l[0] - x_delta, l[1] + y_delta]
            if position[0] <= 0 and 0.7 < abs(y_delta / footprint_to_goal_post.position[0]) < 1.2:
                potential_robot_locations.append(position)

        print("Potential Robot Locations")
        for p in potential_robot_locations:
            print(f" {p}")

        # Valid robot locations (assuming the robots are placed on the bottom line, the line on the opposite side of the field
        # assuming robot is bottom left, because the bottom right line
        valid_robot_transforms = []
        written_robot_transforms = {}
        for id, p in team_info["players"].items():
            position = p["reentryStartingPose"]["translation"]
            axang = p["reentryStartingPose"]["rotation"]
            quaternion = Transformation.get_quaternion_from_axis_angle(axang[0:3], axang[3])
            t = Transformation(position=position, quaternion=quaternion)
            valid_robot_transforms.append(t)
            written_robot_transforms[int(id)] = t

        # Compare the potential robot locations and see which valid robot transformation is closest to the potential robot locations
        closest_valid_robot_transform = None
        closest_valid_robot_transform_distance = 100000
        for transform in valid_robot_transforms:
            pos = transform.position[0:2]
            for potential_location in potential_robot_locations:
                dist_squared = (potential_location[0] - pos[0]) ** 2 + (potential_location[1] - pos[1]) ** 2
                if dist_squared < closest_valid_robot_transform_distance:
                    closest_valid_robot_transform = copy.deepcopy(transform)
                    closest_valid_robot_transform.position = np.array((potential_location[0], potential_location[1], 0))
                    closest_valid_robot_transform_distance = dist_squared

        if closest_valid_robot_transform is None:
            return False

        # Set the location
        current_robot.position = closest_valid_robot_transform.pos_theta
        rospy.loginfo(
            f"Estimated Robot position: {closest_valid_robot_transform.pos_theta} Written Robot Position: {written_robot_transforms[current_robot.robot_id].pos_theta}"
        )
        rospy.loginfo(f"Robot Position Determined, Determining Roles, Position: {current_robot.position}")
        current_robot.reset_initial_position()
        return True

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

            print("  Available Robots", [robot.robot_id for robot in unassigned_robots])
            print("  Available Robot Positions", [robot.position for robot in unassigned_robots])
            print("  Available Roles", available_roles)
            print(
                "  Available Roles Positions",
                [friendly_team.formations["ready"][role][0:2] for role in available_roles],
            )
            while len(unassigned_robots) > 0:
                closest_robot: Optional[Robot] = None
                closest_role_index = 0
                closest_distance = math.inf
                for i in range(len(unassigned_robots)):
                    for j in range(len(available_roles)):
                        distance = np.linalg.norm(
                            np.array(friendly_team.formations["ready"][available_roles[j]][0:2]) - unassigned_robots[i].position[0:2]
                        )
                        if distance < closest_distance:
                            closest_distance = distance
                            closest_robot = unassigned_robots[i]
                            closest_role_index = j

                print(
                    f"  Assigning Robot {closest_robot.robot_id} { closest_robot.position } to {available_roles[closest_role_index].name} with location {friendly_team.formations['ready'][available_roles[closest_role_index]][0:2]}"
                )
                if closest_robot.robot_id == current_robot.robot_id:
                    current_robot.role = available_roles[closest_role_index]
                    print("  Completed Assignment")
                    break
                else:
                    unassigned_robots.pop(unassigned_robots.index(closest_robot))
                    available_roles.pop(closest_role_index)
