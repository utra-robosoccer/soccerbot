import random

from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from matplotlib.path import Path
import matplotlib.patches as patches

from robot import Robot
from ball import Ball
from strategy.dummy_strategy import DummyStrategy
from strategy.team_strategy import TeamStrategy
from strategy.utils import GameProperties, Field
import math
import numpy as np
import copy
from soccer_pycontrol import path

class GameEngine:
    PHYSICS_UPDATE_INTERVAL = 0.1
    STRATEGY_UPDATE_INTERVAL = 50  # Every 5 physics steps
    DISPLAY_UPDATE_INTERVAL = 10  # Every 5 physics steps

    def __init__(self, display=True):
        self.display = display
        # Initialize robots
        self.robots = [
            Robot(robot_id=1, team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                  position=np.array([3.5, 0.0, math.pi])),
            # Robot(robot_id=2, team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
            #       position=np.array([1.5, -1.5, -math.pi])),
            # Robot(robot_id=3, team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
            #       position=np.array([1.5, 1.5, -math.pi])),
            # Robot(robot_id=4, team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY,
            #       position=np.array([0.8, 0.0, -math.pi])),
            # Robot(robot_id=5, team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
            #       position=np.array([-3.5, 0.0, 0])),
            # Robot(robot_id=6, team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
            #       position=np.array([-1.5, -1.5, 0])),
            # Robot(robot_id=7, team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
            #       position=np.array([-1.5, 1.5, 0])),
            # Robot(robot_id=8, team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY,
            #       position=np.array([-0.8, 0.0, 0]))
        ]
        self.ball = Ball(position=np.array([0, 0]))

        self.robots_init = copy.deepcopy(self.robots)
        self.ball_init = copy.deepcopy(self.ball)

        # Initialize display
        # Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf

        if self.display:
            fig = plt.figure(figsize=(9.0, 6.0), dpi=60)
            background = fig.add_axes([0, 0, 1, 1])
            background.axis('equal')
            background.set_xlim([-5, 5])
            background.set_ylim([-3.5, 3.5])
            background.xaxis.set_major_locator(MultipleLocator(1))
            background.yaxis.set_major_locator(MultipleLocator(1))
            background.xaxis.set_minor_locator(MultipleLocator(0.1))
            background.yaxis.set_minor_locator(MultipleLocator(0.1))
            background.grid(which='minor', alpha=0.2)
            background.grid(which='major', alpha=0.5)
            background.add_patch(plt.Rectangle((-4.5, -3 ), 9, 6, alpha=0.1, color='green'))
            background.add_patch(plt.Rectangle((-4.55, -1.3), 0.05, 2.6, color='blue'))
            background.add_patch(plt.Rectangle((4.5, -1.3), 0.05, 2.6, color='blue'))
            background.add_line(plt.Line2D((0, 0), (-3, 3), color='blue'))
            background.add_patch(plt.Circle((-0, 0), 1.3 / 2, fill=None, color='blue'))
            foreground = fig.add_axes([0, 0, 1, 1])
            foreground.set_facecolor((0, 0, 0, 0))


        # Setup the strategy
        self.team1_strategy = TeamStrategy(GameEngine.PHYSICS_UPDATE_INTERVAL * GameEngine.STRATEGY_UPDATE_INTERVAL)
        self.team2_strategy = DummyStrategy()

    def run(self):
        game_period_steps = int(2 * 10 * 60 / GameEngine.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 10 minutes each

        friendly_points = 0
        opponent_points = 0

        for step in range(game_period_steps):
            if step == int(game_period_steps / 2):
                print("Second Half Started: ")
                self.resetRobots()

            self.updateEstimatedPhysics(self.robots, self.ball)

            if step % GameEngine.STRATEGY_UPDATE_INTERVAL == 0:
                self.team1_strategy.update_team_strategy(self.robots, self.ball, GameProperties(0, 1, 0))
                self.team2_strategy.update_team_strategy(self.robots, self.ball, GameProperties(1, 1, 0, opponent_team=True))

            # Check victory condition
            if self.ball.get_position()[0] > 4.5:
                print("Friendly Scores!")
                friendly_points += 1
                self.resetRobots()
            elif self.ball.get_position()[0] < -4.5:
                print("Opponent Scores!")
                opponent_points += 1
                self.resetRobots()

            if self.display and step % GameEngine.DISPLAY_UPDATE_INTERVAL == 0:
                self.displayGameState(self.robots, self.ball, step * GameEngine.PHYSICS_UPDATE_INTERVAL)

        print(F"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        if self.display:
            plt.show()
        return friendly_points, opponent_points

    def displayGameState(self, robots, ball, t=0.0):
        foreground = plt.gcf().axes[1]
        foreground.clear()
        foreground.axis('equal')
        foreground.set_xlim([-5, 5])
        foreground.set_ylim([-3.5, 3.5])
        # Display Robots
        for robot in robots:
            x = robot.get_position()[0]
            y = robot.get_position()[1]
            theta = robot.get_position()[2]

            if robot.team == Robot.Team.OPPONENT:
                color = 'red'
            else:
                color = 'green'
            foreground.add_patch(plt.Circle((x, y), 0.08, color=color))

            arrow_len = 0.3
            arrow_end_x = math.cos(theta) * arrow_len
            arrow_end_y = math.sin(theta) * arrow_len
            foreground.arrow(x, y, arrow_end_x, arrow_end_y, head_width=0.05, head_length=0.1, color=color)

        # Draw ball
        if ball.get_position() is not None:
            x = ball.get_position()[0]
            y = ball.get_position()[1]
            dx = ball.get_velocity()[0]
            dy = ball.get_velocity()[1]
            foreground.add_patch(plt.Circle((x, y), 0.5 / 2 / math.pi, color='black'))
            foreground.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1)

        for robot in self.robots:
            if robot.path is not None:
                verts = []
                for i in range(0,11):
                    path_vert = robot.path.poseAtRatio(i / 10).get_position()
                    verts.append([path_vert[0], path_vert[1]])

                x, y = zip(*verts)
                if robot.team == Robot.Team.FRIENDLY:
                    foreground.plot(x, y, 'g-')
                else:
                    foreground.plot(x, y, 'r-')
        # GUI text
        foreground.text(-3, 4.5, "Time: {0:.6g}".format(t))

        plt.pause(0.001)

    def updateEstimatedPhysics(self, robots, ball):

        # Robot do action in a random priority order
        for robot in sorted(robots, key=lambda _: random.random()):
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
                if ball.kick_timeout == 0:
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
        # assume that robot always know where the ball is
        self.ball.position_is_live_timeout = 10
        self.ball.position = self.ball.get_position() + self.ball.get_velocity() * GameEngine.PHYSICS_UPDATE_INTERVAL

        if not np.array_equal(self.ball.velocity, np.array([0, 0])):
            ball_unit_velocity = self.ball.velocity / np.linalg.norm(self.ball.velocity)

            ball_delta_speed = Ball.FRICTION * GameEngine.PHYSICS_UPDATE_INTERVAL
            ball_speed = np.linalg.norm(self.ball.velocity)
            if ball_speed > ball_delta_speed:
                self.ball.velocity = self.ball.velocity - ball_delta_speed * ball_unit_velocity
            else:
                self.ball.velocity = np.array([0, 0])

    def resetRobots(self):
        self.robots = copy.deepcopy(self.robots_init)
        self.ball = copy.deepcopy(self.ball_init)