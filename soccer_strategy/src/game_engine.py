import random
from vispy import scene, app
from vispy.color import Color

from formation.stationary_formation_strategy import StationaryFormationStrategy
from robot import Robot
from ball import Ball
# from strategy.team_strategy import TeamStrategy
from strategy.stationary_strategy import StationaryStrategy
from formation.normal_formation_strategy import NormalFormationStrategy
from team_data import TeamData2D

import math
import numpy as np
import copy
import _thread

white = Color("#ecf0f1")
gray = Color("#121212")
red = Color("#e74c3c")
blue = Color("#2980b9")
orange = Color("#e88834")
green = Color("#50bb00")
black = Color("#000000")


class Scene:
    def __init__(self, robots, ball):
        self.canvas = scene.SceneCanvas(keys='interactive')
        self.canvas.size = 800, 400
        self.view = self.canvas.central_widget.add_view()
        self.view.bgcolor = black
        # self.view.camera = 'turntable'
        self.view.camera = scene.cameras.panzoom.PanZoomCamera(rect=(-5, -3.5, 10, 7))
        self.canvas.show()
        self.draw_field()
        self.init_actors(robots, ball)

    def draw_field(self):
        field = scene.Rectangle(center=(0, 0), width=10, height=7,
                                color=green, parent=self.view.scene)
        center_circle = scene.Ellipse(center=(0, 0), radius=1.3 / 2, color=green, border_color=white, border_width=2,
                                      parent=self.view.scene)
        center_line = scene.Line(pos=np.array([[0, 3.5], [0, -3.5]]), width=2, color=white, parent=self.view.scene)
        goals = scene.Line(pos=np.array([[-4.5, 1.3], [-4.5, -1.3], [4.5, 1.3], [4.5, -1.3]]), connect='segments',
                           width=2, color=white, parent=self.view.scene)

    def init_actors(self, robots, ball):
        self.ball = scene.Ellipse(center=(ball.position[0], ball.position[1]), radius=0.1,
                                  color=blue, parent=self.view.scene)
        self.robots = []
        for robot in robots:
            color = red if robot.team == Robot.Team.OPPONENT else white
            self.robots.append({"body": scene.Ellipse(center=(ball.position[0], ball.position[1]), radius=0.1,
                                                      color=color,
                                                      parent=self.view.scene),
                                "arrow": scene.Arrow(pos=np.array([[0, 0], [0, 0]]), width=1, color=color,
                                                     parent=self.view.scene),
                                "path": scene.Arrow(pos=None, width=1, color=color, connect='strip',
                                                    parent=self.view.scene),
                                "vision_cone": scene.Arrow(pos=None, width=1, color=color, connect='strip',
                                                           parent=self.view.scene),
                                "id": scene.Text(text="id", parent=self.view.scene, pos=(0, 0))})

        self.field_vectors = scene.Arrow(pos=None, width=1, color=blue, connect='segments', arrows=None,
                                         arrow_color=blue, arrow_type='triangle_30', arrow_size=7,
                                         parent=self.view.scene)

    def update(self, robots, ball, t=0.0):
        for i in range(len(robots)):
            x = robots[i].get_position()[0]
            y = robots[i].get_position()[1]
            self.robots[i]['body'].center = (x, y)
            self.robots[i]['id'].pos = (x, y)
            self.robots[i]['id'].text = str(robots[i].robot_id)

            theta = robots[i].get_position()[2]
            arrow_len = 0.3
            arrow_end_x = math.cos(theta) * arrow_len
            arrow_end_y = math.sin(theta) * arrow_len
            direction = np.array([arrow_end_x, arrow_end_y])
            direction = direction / np.linalg.norm(direction)
            self.robots[i]['arrow'].set_data(pos=np.array([[x, y], [x + arrow_end_x, y + arrow_end_y]]))

            robot_pos = np.array([x, y])
            theta = -Robot.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_right = np.matmul(R, direction) * Robot.ObservationConstants.VISION_RANGE
            theta = Robot.ObservationConstants.FOV / 2
            c, s = np.cos(theta), np.sin(theta)
            R = np.array(((c, -s), (s, c)))
            vision_cone_left = np.matmul(R, direction) * Robot.ObservationConstants.VISION_RANGE
            self.robots[i]['vision_cone'].set_data(
                pos=np.array([robot_pos + vision_cone_left, robot_pos, vision_cone_right + robot_pos]))

            if robots[i].path is not None:
                verts = []
                for j in range(0, 11):
                    path_vert = robots[i].path.poseAtRatio(j / 10).get_position()
                    verts.append([path_vert[0], path_vert[1]])
                self.robots[i]['path'].set_data(pos=np.array(verts))

        if ball.get_position() is not None:
            x = ball.get_position()[0]
            y = ball.get_position()[1]
            dx = ball.get_velocity()[0]
            dy = ball.get_velocity()[1]
            self.ball.center = (x, y)

        self.canvas.update()

    def plot_vectors(self, field_vectors):
        arrows = []
        pos = []
        for k in range(0, len(field_vectors)):
            if field_vectors[k] is not None:
                for i in range(0, len(field_vectors[k][0])):
                    x1 = field_vectors[k][0][i][0]
                    y1 = field_vectors[k][0][i][1]
                    x2 = x1 + field_vectors[k][1][i][0]
                    y2 = y1 + field_vectors[k][1][i][1]
                    arrows.append([x1, y1, x2, y2])
                    pos.append([x1, y1])
                    pos.append([x2, y2])
        self.field_vectors.set_data(pos=np.array(pos), arrows=np.array(arrows))


class Team:
    def __init__(self, robots, strategy):
        # TODO better way to construct team - predefined start positions etc?
        # TODO seperate subclass for opponent and friendly team?
        self.robots = robots if robots is not None else []
        self.observed_ball = Ball(None, '2D')
        self.team_data = TeamData2D()
        self.strategy = strategy(self.team_data)

    def update(self):
        self.observe_ball()
        for robot in self.robots:
            self.team_data.team_data_callback([robot, self.observed_ball])
        for robot in self.robots:
            self.strategy.update_strategy(robot)

    # this should prolly be happening more on the robot side than the game engine, but its weird
    def observe_ball(self):
        for robot in self.robots:
            if robot.observed_ball.position is not None:
                self.observed_ball.position = robot.observed_ball.position


# TODO seperate the physics from the driver class?

class GameEngine:
    PHYSICS_UPDATE_INTERVAL = 0.1
    STRATEGY_UPDATE_INTERVAL = 10  # Every 5 physics steps
    DISPLAY_UPDATE_INTERVAL = 1  # Every 5 physics steps

    def __init__(self, display=True):
        self.display = display
        # Initialize robots
        self.ball = Ball(np.array([0, 2]), '2D')

        self.robots = [
            Robot(robot_id=1, team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                  position=np.array([3.5, 0.0, math.pi])),
            Robot(robot_id=2, team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([1.5, -1.5, -math.pi])),
            Robot(robot_id=3, team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([1.5, 1.5, -math.pi])),
            Robot(robot_id=4, team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                  position=np.array([0.8, 0.0, -math.pi])),
            Robot(robot_id=5, team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                  position=np.array([-3.5, 0.0, 0])),
            Robot(robot_id=6, team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([-1.5, -1.5, 0])),
            Robot(robot_id=7, team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                  position=np.array([-1.5, 1.5, 0])),
            Robot(robot_id=8, team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                  position=np.array([-0.8, 0.0, 0]))]

        # self.team1_strategy = TeamStrategy(GameEngine.PHYSICS_UPDATE_INTERVAL * GameEngine.STRATEGY_UPDATE_INTERVAL)
        # self.team1_strategy = DummyStrategy()
        # self.team2_strategy = StationaryStrategy()
        #
        # # # Setup the strategy
        # self.team1_strategy_rtval = None
        # self.team2_strategy_rtval = None

        self.friendly_team = Team(self.robots[:4], NormalFormationStrategy)
        self.opponent_team = Team(self.robots[4:], StationaryFormationStrategy)

        self.robots_init = copy.deepcopy(self.robots)
        self.ball_init = copy.deepcopy(self.ball)

        # Initialize display
        # Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf
        if self.display:
            self.scene = Scene(self.robots, self.ball)

    def run_loop(self):
        _thread.start_new_thread(self.run, ())
        app.run()

    # TODO it is slow because it is re-pathing whenever the ball changes its position
    def run(self):
        game_period_steps = int(2 * 10 * 60 / GameEngine.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 10 minutes each
        friendly_points = 0
        opponent_points = 0

        for step in range(game_period_steps):
            if step == int(game_period_steps / 2):
                print("Second Half Started: ")
                self.reset_robots()

            self.update_estimated_physics(self.robots, self.ball)

            if step % GameEngine.STRATEGY_UPDATE_INTERVAL == 0:
                self.friendly_team.update()
                self.opponent_team.update()
            #     self.team1_strategy_rtval = self.team1_strategy.update_team_strategy(self.robots, self.friendly_team.observed_ball,
            #                                                                                  GameProperties(0, 1, 0))
            #     self.team2_strategy_rtval = self.team2_strategy.update_team_strategy(self.robots, self.opponent_team.observed_ball,
            #                                                                                  GameProperties(1, 1, 0,
            #                                                                                     opponent_team=True))

            # Check victory condition
            if self.ball.get_position()[0] > 4.5:
                print("Friendly Scores!")
                friendly_points += 1
                self.reset_robots()
            elif self.ball.get_position()[0] < -4.5:
                print("Opponent Scores!")
                opponent_points += 1
                self.reset_robots()

            if self.display and step % GameEngine.DISPLAY_UPDATE_INTERVAL == 0:
                self.display_game_states()

        print(F"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        return friendly_points, opponent_points

    def display_game_states(self):
        self.scene.update(self.robots, self.ball)
        # if self.team1_strategy_rtval is not None:
        #     if "potential_field_vectors" in self.team1_strategy_rtval:
        #         field_vectors = self.team1_strategy_rtval["potential_field_vectors"]
        #         self.scene.plot_vectors(field_vectors)

    def update_estimated_physics(self, robots, ball):  # TODO why is ball passed here?
        # Robot do action in a random priority order
        for robot in robots:
            # for robot in sorted(robots, key=lambda _: random.random()):
            robot.observe_ball(ball)  # TODO better place for this?
            if robot.status == Robot.Status.WALKING:
                robot.path_time = robot.path_time + GameEngine.PHYSICS_UPDATE_INTERVAL
                update_position_transformation = robot.path.estimatedPositionAtTime(robot.path_time)
                update_position = robot.transformation_to_position(update_position_transformation)

                # if done walking
                # if robot.robot_id == 4:
                #     print(robot.path.isFinished(robot.path_time))
                #     print(str(robot.path_time) + " - " + str(robot.path.duration()))
                if robot.path.isFinished(robot.path_time):
                    robot.status = Robot.Status.READY
                    continue

                robot.position = update_position

            elif robot.status == Robot.Status.KICKING:
                if ball.kick_timeout == 0 and robot.can_kick(ball):
                    ball.velocity = robot.kick_velocity
                    ball.kick_timeout = 5
                robot.status = Robot.Status.READY

        # Ball

        if ball.kick_timeout > 0:
            ball.kick_timeout = ball.kick_timeout - 1

        # TODO If ball hits a person, bounce
        self.update_average_ball_position()

    def update_average_ball_position(self):
        # update ball position
        self.ball.position_is_live_timeout = 10
        self.ball.position = self.ball.get_position() + self.ball.get_velocity() * GameEngine.PHYSICS_UPDATE_INTERVAL

        # slow down ball with friction
        if not np.array_equal(self.ball.velocity, np.array([0, 0])):
            ball_unit_velocity = self.ball.velocity / np.linalg.norm(self.ball.velocity)

            ball_delta_speed = Ball.FRICTION * GameEngine.PHYSICS_UPDATE_INTERVAL
            ball_speed = np.linalg.norm(self.ball.velocity)
            if ball_speed > ball_delta_speed:
                self.ball.velocity = self.ball.velocity - ball_delta_speed * ball_unit_velocity
            else:
                self.ball.velocity = np.array([0, 0])

    def reset_robots(self):
        self.robots = copy.deepcopy(self.robots_init)
        self.friendly_team = Team(self.robots[:4], NormalFormationStrategy)
        self.opponent_team = Team(self.robots[4:], StationaryFormationStrategy)
        self.ball = copy.deepcopy(self.ball_init)
