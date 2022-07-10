import math

# import rospy
# rospy only used here for rospy.loginfo
import time

import rospy
from strategy.interfaces.actions import Actions
from strategy.interfaces.evaluations import Evaluations
from strategy.strategy import Strategy, get_back_up, update_average_ball_position
from strategy.utils import *
from team import Team

from soccer_msgs.msg import GameState

GRADIENT_UPDATE_INTERVAL_LENGTH = 0.5

ALPHA = 0.5
BETA = 0.5
EPS = 0.5


class Thresholds:
    POSSESSION = 0.2  # How close player has to be to ball to have possession
    GOALIE_ANGLE = 5  # How close goalie has to be to defense line
    PASS = -1  # Maximum distance between players trying to pass to each other
    OBSTACLE = 1  # Size of player obstacles
    PASSING = 2  # distance for obstacle detection when moving to a position


class StrategyDummy(Strategy):
    def __init__(self):
        self.time_of_end_of_action = rospy.Time.now()
        self.update_frequency = 1
        super(StrategyDummy, self).__init__()

    @get_back_up
    @update_average_ball_position
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)

        this_robot = self.get_current_robot(friendly_team)

        if this_robot.status in [
            Robot.Status.WALKING,
            Robot.Status.KICKING,
            Robot.Status.FALLEN_BACK,
            Robot.Status.FALLEN_FRONT,
            Robot.Status.FALLEN_SIDE,
            Robot.Status.TRAJECTORY_IN_PROGRESS,
        ]:
            self.time_of_end_of_action = rospy.Time.now()
            return

        # If the ball has been seen in the last 2 seconds
        if (
            rospy.Time.now() - this_robot.observed_ball.last_observed_time_stamp < rospy.Duration(2)
            and friendly_team.average_ball_position.position is not None
        ):

            # generate goal pose
            goal_position = friendly_team.enemy_goal_position
            ball = friendly_team.average_ball_position

            current_closest = Evaluations.who_has_the_ball(friendly_team.robots, ball)  # Guess who has the ball
            if current_closest is None:
                pass
            elif current_closest.robot_id == this_robot.robot_id:
                if this_robot.can_kick(ball, goal_position):
                    if this_robot.status in [Robot.Status.READY]:

                        delta = goal_position - ball.position[0:2]
                        unit = delta / np.linalg.norm(delta)

                        this_robot.status = Robot.Status.KICKING
                        this_robot.set_kick_velocity(unit * this_robot.max_kick_speed)
                        this_robot.kick()
                else:
                    # Ball localized, move to ball
                    if (rospy.Time.now() - this_robot.navigation_goal_localized_time) < rospy.Duration(
                        2
                    ) and this_robot.status != Robot.Status.WALKING:
                        rospy.loginfo("Navigation to ball")
                        Actions.navigate_to_scoring_position(this_robot, np.array(ball.position[0:2]), goal_position)

            pass

        # If the ball hasn't been seen in 10 seconds
        elif rospy.Time.now() - this_robot.observed_ball.last_observed_time_stamp > rospy.Duration(
            10
        ) and rospy.Time.now() - self.time_of_end_of_action > rospy.Duration(10):
            if this_robot.status not in [Robot.Status.WALKING, Robot.Status.KICKING]:
                player_angle = this_robot.position[2]
                player_position = this_robot.position[0:2]

                # Haven't seen the ball timeout
                rospy.loginfo(
                    f"Player {this_robot.robot_id}: Rotating to locate ball. Time of End of Action {self.time_of_end_of_action}, Last Observed Time Stamp {this_robot.observed_ball.last_observed_time_stamp}"
                )

                turn_position = [player_position[0], player_position[1], player_angle + math.pi * 0.8]
                this_robot.set_navigation_position(turn_position)
