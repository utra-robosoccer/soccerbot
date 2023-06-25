import os
import subprocess
import threading
from unittest.mock import MagicMock, patch

import controller
import numpy as np
from geometry import distance2
from referee import Referee

from soccer_strategy.game_engine_2d import GameEngine2D
from soccer_strategy.robot_controlled import RobotControlled

GAME_INTERRUPTIONS = {
    "DIRECT_FREEKICK": "direct free kick",
    "INDIRECT_FREEKICK": "indirect free kick",
    "PENALTYKICK": "penalty kick",
    "CORNERKICK": "corner kick",
    "GOALKICK": "goal kick",
    "THROWIN": "throw in",
}


class FakeSupervisor:
    pass


class Referee2D(Referee):
    class BallTranslation2D:
        def __init__(self, game_engine_2d: GameEngine2D):
            self.game_engine_2d = game_engine_2d
            pass

        def getSFVec3f(self):
            p = self.game_engine_2d.ball.position
            return [p[0], p[1], 0]

        def setSFVec3f(self, target_location):
            self.game_engine_2d.ball.position = np.array([target_location[0], target_location[1]])

    def __init__(self, game_engine_2d: GameEngine2D):
        os.system("/bin/bash -c 'killall python3 || echo 'No Python Executables running''")
        os.system("/bin/bash -c 'killall /usr/bin/java || echo 'No Java Executables running''")

        os.environ["JAVA_HOME"] = "/usr"
        os.environ["GAME_CONTROLLER_HOME"] = os.path.dirname(os.path.realpath(__file__)) + "/../../../external/GameController/"
        controller.Supervisor = FakeSupervisor

        main_loop_original = self.main_loop
        clean_exit_original = self.clean_exit
        self.game_engine_2d = game_engine_2d
        self.main_loop = lambda: None
        self.clean_exit = lambda: None

        super().__init__()

        self.main_loop = main_loop_original
        self.clean_exit = clean_exit_original

        self.robot_last_position = {}

        self.game.ball_position = self.game_engine_2d.ball.position

        self.game.ball_translation = Referee2D.BallTranslation2D(game_engine_2d)

    def list_player_solids(self, *args, **kwargs):
        pass

    def update_team_contacts(self, team):
        early_game_interruption = self.is_early_game_interruption()
        color = team.color

        for number, player in team.players.items():
            if color == "red":
                team_game_engine_2d = self.game_engine_2d.team1
            else:
                team_game_engine_2d = self.game_engine_2d.team2

            robot_game_engine_2d = team_game_engine_2d.robots[int(number) - 1]

            other_robot_positions = []
            for team in [self.game_engine_2d.team1, self.game_engine_2d.team2]:
                for robot in team.robots:
                    if robot != robot_game_engine_2d:
                        other_robot_positions.append(robot.position)

            ball_position = self.game_engine_2d.ball.position

            robot_position = robot_game_engine_2d.position
            robot_velocity = np.array([0, 0])
            if robot_game_engine_2d in self.robot_last_position:
                last_position = self.robot_last_position[robot_game_engine_2d]
                robot_velocity = robot_game_engine_2d.position - last_position

            self.robot_last_position[robot_game_engine_2d] = robot_game_engine_2d.position

            l1 = len(player["velocity_buffer"])  # number of iterations
            l2 = len(player["velocity_buffer"][0])  # should be 6 (velocity vector size)
            player["velocity_buffer"][int(self.sim_time.get_ms() / self.time_step) % l1] = [robot_velocity[0], robot_velocity[1], 0, 0, 0, 0]
            sum = [0] * l2
            for v in player["velocity_buffer"]:
                for i in range(l2):
                    sum[i] += v[i]
            player["velocity"] = [s / l1 for s in sum]

            # Calculate contact point with other robots
            contact_points = []
            for other_robot_position in other_robot_positions:
                if np.linalg.norm(robot_position - other_robot_position) < (RobotControlled.BODY_WIDTH * 2):
                    contact_points.append(list(0.5 * robot_position + 0.5 * other_robot_position))

            # Calculate contact point with ball
            ball_position_3d = np.array([ball_position[0], ball_position[1], 0])
            if np.linalg.norm(robot_position - ball_position_3d) < (RobotControlled.BODY_WIDTH + 0.07):
                contact_points.append(list(ball_position_3d))

            n = len(contact_points)
            player["contact_points"] = []
            player["asleep"] = False
            player["position"] = robot_position
            player["outside_circle"] = True  # true if fully outside the center cicle
            player["outside_field"] = True  # true if fully outside the field
            player["inside_field"] = True  # true if fully inside the field
            player["on_outer_line"] = False  # true if robot is partially on the line surrounding the field
            player["inside_own_side"] = True  # true if fully inside its own side (half field side)
            player["outside_goal_area"] = True  # true if fully outside of any goal area
            player["outside_penalty_area"] = True  # true if fully outside of any penalty area
            outside_turf = False  # true if fully outside turf

            for i in range(n):
                point = contact_points[i]

                if not early_game_interruption and point == ball_position:  # ball contact
                    if self.game.ball_first_touch_time == 0:
                        self.game.ball_first_touch_time = self.sim_time.get_ms()
                    self.game.ball_last_touch_time = self.sim_time.get_ms()
                    if self.game.penalty_shootout_count >= 10:  # extended penalty shootout
                        self.game.penalty_shootout_time_to_touch_ball[self.game.penalty_shootout_count - 10] = 60 - self.game.state.seconds_remaining
                    if self.game.ball_last_touch_team != color or self.game.ball_last_touch_player_number != int(number):
                        self.game.set_ball_touched(color, int(number))
                        self.game.ball_last_touch_time_for_display = self.sim_time.get_ms()
                        action = "kicked" if self.game.kicking_player_number is None else "touched"
                        self.logger.info(f"Ball {action} by {color} player {number}.")
                        if self.game.kicking_player_number is None:
                            self.game.kicking_player_number = int(number)
                    elif self.sim_time.get_ms() - self.game.ball_last_touch_time_for_display >= 1000:
                        # dont produce too many touched messages
                        self.game.ball_last_touch_time_for_display = self.sim_time.get_ms()
                        self.logger.info(f"Ball touched again by {color} player {number}.")
                    step = self.game.state.secondary_state_info[1]
                    if step != 0 and self.game.state.secondary_state[6:] in GAME_INTERRUPTIONS:
                        self.game_interruption_touched(team, number)
                    continue
                # the robot touched something else than the ball or the ground
                player["contact_points"].append(point)  # this list will be checked later for robot-robot collisions

                if distance2(point, [0, 0]) < self.field.circle_radius:
                    player["outside_circle"] = False
                if self.field.point_inside(point, include_turf=True):
                    outside_turf = False
                if self.field.point_inside(point):
                    player["outside_field"] = False
                    if abs(point[0]) > self.field.size_x - self.field.penalty_area_length and abs(point[1]) < self.field.penalty_area_width / 2:
                        player["outside_penalty_area"] = False
                        if abs(point[0]) > self.field.size_x - self.field.goal_area_length and abs(point[1]) < self.field.goal_area_width / 2:
                            player["outside_goal_area"] = False
                    if not self.field.point_inside(point, include_turf=False, include_border_line=False):
                        player["on_outer_line"] = True
                else:
                    player["inside_field"] = False
                if self.game.side_left == (self.game.red.id if color == "red" else self.game.blue.id):
                    if point[0] > -self.field.line_half_width:
                        player["inside_own_side"] = False
                else:
                    if point[0] < self.field.line_half_width:
                        player["inside_own_side"] = False

            if not player["on_outer_line"]:
                player["on_outer_line"] = not (player["inside_field"] or player["outside_field"])
            if outside_turf:
                if player["left_turf_time"] is None:
                    player["left_turf_time"] = self.sim_time.get_ms()
            else:
                player["left_turf_time"] = None

    def place_player_at_penalty(self, player, team, number):
        # TODO write this
        pass
