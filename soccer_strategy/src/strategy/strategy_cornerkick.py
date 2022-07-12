import rospy
from strategy.interfaces.actions import Actions
from strategy.strategy import Strategy, get_back_up
from strategy.utils import *
from team import Team

from soccer_msgs.msg import GameState


class StrategyCornerkick(Strategy):
    def update_next_strategy(self, friendly_team: Team, opponent_team: Team, game_state: GameState):
        super().update_next_strategy(friendly_team, opponent_team, game_state)
        this_robot = self.get_current_robot(friendly_team)

        # assign one robot to always do the cornerkicks
        # TODO make sure this works when the ball starts on any side

        CORNERKICK_ROBOT = 2
        CORNERKICK_TARGET = np.array([3, 0])

        if this_robot.robot_id == CORNERKICK_ROBOT:
            if rospy.Time.now() - this_robot.observed_ball.last_observed_time_stamp < rospy.Duration(2):
                goal_position = CORNERKICK_TARGET
                ball = this_robot.observed_ball
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
            else:
                rospy.loginfo("Navigating to corner")
                Actions.navigation_to_position(this_robot, np.array([2, 2]))
