import math

import numpy as np
import rospy

from soccer_msgs.msg import GameState
from soccer_strategy.robot import Robot
from soccer_strategy.strategy.strategy import (
    Strategy,
    get_back_up,
    update_average_ball_position,
)
from soccer_strategy.strategy.utils import Utility
from soccer_strategy.team import Team


class StrategyDummy(Strategy):
    """
    Very basic strategy used, simply kick and go
    """

    def __init__(self):
        self.time_of_end_of_action = rospy.Time.now()
        self.update_frequency = 1
        super(StrategyDummy, self).__init__()

    @get_back_up
    @update_average_ball_position
    def step_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        """
        Looks for the ball if it doesn't see the ball, if it sees, then kick the ball towards the net

        :param friendly_team: The friendly Team
        :param opponent_team: The opponent Team
        :param game_state: The GameState from the game controller
        """

        super().step_strategy(friendly_team, opponent_team, game_state)

        this_robot = self.get_current_robot(friendly_team)

        if this_robot.status in [
            Robot.Status.WALKING,
            Robot.Status.KICKING,
            Robot.Status.FALLEN_BACK,
            Robot.Status.FALLEN_FRONT,
            Robot.Status.FALLEN_SIDE,
            Robot.Status.GETTING_BACK_UP,
        ]:
            self.time_of_end_of_action = rospy.Time.now()
            return

        # If the ball has been seen in the last 2 seconds
        if friendly_team.observed_ball is not None:

            # generate goal pose
            goal_position = friendly_team.enemy_goal_position
            ball = friendly_team.observed_ball

            current_closest = self.who_has_the_ball(friendly_team.robots, ball)  # Guess who has the ball
            if current_closest is None:
                pass
            elif current_closest.robot_id == this_robot.robot_id:
                if this_robot.can_kick(ball, goal_position):
                    if this_robot.status in [Robot.Status.READY]:

                        delta = goal_position - ball.position[0:2]
                        unit = delta / np.linalg.norm(delta)

                        this_robot.status = Robot.Status.KICKING
                        this_robot.kick(kick_velocity=unit * this_robot.max_kick_speed)
                else:
                    if this_robot.status == Robot.Status.WALKING:
                        # TODO more intelligent strategy where other robot goes for the ball if it is not going
                        return

                    # Ball localized, move to ball
                    if (rospy.Time.now() - this_robot.robot_focused_on_ball_time) < rospy.Duration(2):
                        blocked_position = Utility.is_obstacle_blocking(this_robot, this_robot.position[0:2], ball.position)
                        if blocked_position is not None:
                            optimal_pos = Utility.optimal_position_to_navigate_if_blocking(
                                this_robot.position[0:2], np.array(ball.position[0:2]), blocked_position, this_robot.BODY_WIDTH
                            )
                            Utility.navigate_to_position_with_offset(this_robot, optimal_pos, np.array(ball.position[0:2]), 0)
                        else:
                            rospy.loginfo("Navigation to ball")
                            blocked_position = Utility.is_obstacle_blocking(this_robot, ball.position, goal_position)
                            if blocked_position is not None:
                                optimal_pos = Utility.optimal_position_to_navigate_if_blocking(
                                    ball.position[0:2], goal_position[0:2], blocked_position, this_robot.BODY_WIDTH
                                )
                                Utility.navigate_to_position_with_offset(this_robot, np.array(ball.position[0:2]), optimal_pos)
                            else:
                                Utility.navigate_to_scoring_position(this_robot, np.array(ball.position[0:2]), goal_position)
                    elif this_robot.observed_ball is None or (rospy.Time.now() - this_robot.observed_ball.last_observed_time_stamp) > rospy.Duration(
                        2
                    ):
                        # Other robot sees the ball but this robot is not seeing the ball, then turn to face the ball
                        player_angle = this_robot.position[2]
                        player_position = this_robot.position[0:2]

                        # Haven't seen the ball timeout
                        rospy.loginfo(f"Player {this_robot.robot_id}: Rotating to locate ball that other robot found {ball.position}")
                        robot_to_ball = ball.position - player_position
                        angle = np.arctan2(robot_to_ball[1], robot_to_ball[0])

                        turn_position = np.array([player_position[0], player_position[1], angle])
                        this_robot.set_navigation_position(turn_position)
                    else:
                        # Waiting to focus on ball
                        pass

        # If the ball hasn't been seen in 10 seconds
        elif (
            friendly_team.observed_ball is None
            or rospy.Time.now() - friendly_team.observed_ball.last_observed_time_stamp
            > rospy.Duration(rospy.get_param("delay_before_rotate_to_search_ball", 10))
        ) and rospy.Time.now() - self.time_of_end_of_action > rospy.Duration(rospy.get_param("delay_before_rotate_to_search_ball", 10)):
            if this_robot.status not in [Robot.Status.WALKING, Robot.Status.KICKING]:
                player_angle = this_robot.position[2]
                player_position = this_robot.position[0:2]

                # Haven't seen the ball timeout
                rospy.loginfo(
                    f"Player {this_robot.robot_id}: Rotating to locate ball. Time of End of Action {self.time_of_end_of_action}, Last Observed Time Stamp {friendly_team.observed_ball.last_observed_time_stamp if friendly_team.observed_ball is not None else 0}"
                )

                turn_position = np.array([player_position[0], player_position[1], player_angle + math.pi * 0.45])
                this_robot.set_navigation_position(turn_position)
