import math
import numpy as np
import rospy

import soccer_strategy.src.config as config
from soccer_strategy.src.strategy.strategy import Strategy
from soccer_msgs.msg import GameState
from soccer_strategy.src.robot import Robot

HAVENT_SEEN_THE_BALL_TIMEOUT = 10
class DummyStrategy(Strategy):

    def __init__(self):
        self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
        super(DummyStrategy, self).__init__()

    @staticmethod
    def generate_goal_position(ball, teamcolor, is_first_half, secondaryState):
        is_penalty_shoot = (secondaryState == GameState.STATE_PENALTYSHOOT)
        goal_position = config.position_map_goal(config.GOAL_POSITION, teamcolor, is_first_half, is_penalty_shoot)

        if abs(ball.get_position()[1]) < 1.0:
            goal_position[1] = ball.get_position()[1]

        return goal_position

    def update_next_strategy(self, friendly, opponent, ball, teamcolor, is_first_half, secondaryState):
        if self.check_ball_avaliable(ball):
            self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT

            # generate goal pose
            goal_position = self.generate_goal_position(ball, teamcolor, is_first_half, secondaryState)

            if abs(ball.get_position()[1]) < 3.5 and abs(ball.get_position()[0]) < 5:

                current_closest = self.who_has_the_ball(friendly, ball) # Guess who has the ball
                if current_closest is not None: #and current_closest.send_nav:

                    # generate destination pose
                    ball_position = ball.get_position()
                    player_position = current_closest.get_position()[0:2]
                    player_angle = current_closest.get_position()[2]

                    diff = ball_position - goal_position
                    diff_unit = diff / np.linalg.norm(diff)
                    diff_angle = math.atan2(-diff_unit[1], -diff_unit[0])

                    distance_of_player_goal_to_ball = 0.1
                    destination_position = ball_position + diff_unit * distance_of_player_goal_to_ball

                    navigation_bias = 1
                    diff = destination_position - player_position
                    destination_position_biased = player_position + diff * navigation_bias

                    destination_position_biased = [destination_position_biased[0], destination_position_biased[1], diff_angle]

                    # print("Position of closest player")
                    # print(player_position)
                    # print("Ball Position")
                    # print(ball_position)
                    # print("Destination Position")
                    # print(destination_position)
                    # print("Distance between player and ball")
                    # print(distance_of_player_to_ball)

                    player_vector = [math.cos(player_angle), math.sin(player_angle)]
                    player_to_ball_vector = ball_position - player_position
                    cross = np.cross(player_to_ball_vector, player_vector)

                    distance_of_player_to_ball = np.linalg.norm(player_position - ball_position)
                    if distance_of_player_to_ball < 0.18 and abs(cross) > 0.15:
                        print("robot ball ange too large, unable to kick")

                    if distance_of_player_to_ball < 0.18 and abs(cross) < 0.15:
                        if cross > 0.03:
                            # right foot
                            current_closest.kick_with_right_foot = True
                        else:
                            current_closest.kick_with_right_foot = False

                        delta = goal_position - ball.get_position()
                        unit = delta / np.linalg.norm(delta)

                        current_closest.status = Robot.Status.KICKING
                        current_closest.set_kick_velocity(unit * current_closest.max_kick_speed)
                    else:
                        current_closest.set_navigation_position(destination_position_biased)
                        pass

        else:
            # If player is not facing the right direction, and not seeing the ball, then face the goal
            self.havent_seen_the_ball_timeout = self.havent_seen_the_ball_timeout - 1
            for player in friendly:
                if player.status != Robot.Status.READY:
                    continue

                player_angle = player.get_position()[2]
                player_position = player.get_position()[0:2]

                # Haven't seen the ball timeout
                if self.havent_seen_the_ball_timeout == 0:
                    rospy.loginfo("Havent seen the ball for a while. Rototating robot " + player.robot_name)
                    self.havent_seen_the_ball_timeout = HAVENT_SEEN_THE_BALL_TIMEOUT
                    turn_position = [player_position[0], player_position[1], player_angle + math.pi]
                    player.set_navigation_position(turn_position)


        # If the robot is walking and a detected obstacle in the direction of the robot
        # for player in friendly:
        #     if player.status == Robot.Status.WALKING:
        #         player_angle = player.get_position()[2]
        #         player_position = player.get_position()[0:2]
        #         player_vector = [math.cos(player_angle), math.sin(player_angle)]
        #
        #         obstacles = player.get_detected_obstacles()
        #
        #         for obs in obstacles:
        #             obs_position = np.array(obs[0:2])
        #
        #             if np.linalg.norm(obs_position - player_position) > 0.5:
        #                 continue
        #
        #             player_to_ball_vector = obs_position - player_position
        #             cross = float(np.cross(player_to_ball_vector, player_vector))
        #
        #             if abs(cross) > 0.15 * 2: # TODO tune
        #                 continue
        #
        #             player.terminate_walking_publisher.publish()
        #             player.status = Robot.Status.READY
        #             rospy.sleep(0.5)
        #             player.completed_trajectory_publisher.publish(True)