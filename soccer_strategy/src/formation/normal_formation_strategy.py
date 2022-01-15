import numpy as np
import math
import rospy

from robot_ros import Robot
from formation.formation_strategy import FormationStrategy, Formations


HAVENT_SEEN_THE_BALL_TIMEOUT = 10

class NormalFormationStrategy(FormationStrategy):
    def __init__(self, team_data):
        super().__init__(Formations.DEFENSIVE, team_data)

    # something so there can be different formation deciders like a super defensive biased one
    def decide_formation(self):
        if self.team_data.ball.is_known() and self.team_data.ball.position[0] < 0:  # ball in oppents side
            return Formations.ATTACKING
        else:
            return Formations.DEFENSIVE

    # this can be something to help implement more decisions in a better way?
    def act_individual(self, robot):
        if self.team_data.ball.is_known():
            self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT

            # generate goal pose
            goal_position = self.generate_goal_position(ball, game_properties)

            if abs(self.team_data.ball.position[1]) < 3.5 and abs(self.team_data.ball.position[0]) < 5:

                if robot.robot_id == self.formation.closest_position(self.team_data.ball.position):

                    # generate destination pose
                    ball_position = self.team_data.ball.position
                    player_position = robot.get_position()[0:2]
                    player_angle = robot.get_position()[2]

                    diff = ball_position - goal_position
                    diff_unit = diff / np.linalg.norm(diff)
                    diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

                    distance_of_player_goal_to_ball = 0.1
                    destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball

                    navigation_bias = 1
                    diff = destination_position - player_position
                    # nav bias offset nav goal to be behind the ball
                    destination_position_biased = player_position + diff * navigation_bias

                    # nav goal behind the ball
                    destination_position_biased = [destination_position_biased[0], destination_position_biased[1],
                                                   diff_angle]

                    # difference between robot angle and nav goal angle
                    nav_angle__diff = math.atan2(math.sin(player_angle - diff_angle),
                                                 math.cos(player_angle - diff_angle))

                    distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)
                    if distance_of_player_to_ball < 0.18 and abs(nav_angle__diff) > 0.15:
                        print("robot ball angle too large, unable to kick " + str(abs(nav_angle__diff)))

                    if distance_of_player_to_ball < 0.18 and abs(
                            nav_angle__diff) < 0.15 and robot.path.isFinished(robot.path_time):
                        if nav_angle__diff > 0.03:
                            # right foot
                            robot.kick_with_right_foot = True
                        else:
                            robot.kick_with_right_foot = False

                        delta = goal_position - self.team_data.ball.position
                        unit = delta / np.linalg.norm(delta)

                        robot.status = Robot.Status.KICKING
                        robot.set_kick_velocity(unit * robot.max_kick_speed)
                    else:
                        robot.set_navigation_position(destination_position_biased)

        else:
            # If player is not facing the right direction, and not seeing the ball, then face the goal
            self.havent_seen_the_ball_timeout = self.havent_seen_the_ball_timeout - 1

            if robot.status == Robot.Status.READY:
                player_angle = robot.get_position()[2]
                player_position = robot.get_position()[0:2]

                # Haven't seen the ball timeout
                if self.havent_seen_the_ball_timeout == 0:
                    rospy.loginfo("Havent seen the ball for a while. Rototating robot " + robot.robot_name)
                    self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
                    turn_position = [player_position[0], player_position[1], player_angle + math.pi]
                    robot.set_navigation_position(turn_position)





        #
        # # a decision maker class so these can be strung together and more advacned oeprations?
        # if self.team_data.ball.is_known() and robot.robot_id == self.formation.closest_position(self.team_data.ball.position):
        #     # have actions class with subclasses
        #     # robot.do_now(Actions.MOVE_TO, [ball])
        #     #print("robot ", robot.robot_id, " will move to ball")
        #     goal = [self.team_data.ball.position[0], self.team_data.ball.position[1], 3.14]
        #     if not np.allclose(goal, robot.goal_position):
        #         robot.set_navigation_position(goal)
        #     return
        # else:
        #     # robot.do_now(Actions.MOVE_TO, [self.formation.positions[robot.robot_id])
        #     #print("robot ", robot.robot_id, " will move to position")
        #     goal = [self.formation.positions[robot.robot_id-1].center[0], self.formation.positions[robot.robot_id -1].center[1], 3.14]
        #     #TODO add this if to robot.set_navigation_position
        #     if not np.allclose(goal, robot.goal_position):
        #         robot.set_navigation_position(goal)
        #     return