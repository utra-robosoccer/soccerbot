#!/usr/bin/env python3
import copy
import math

# TODO change to pyqt5 for faster plotting
import numpy as np
from matplotlib.ticker import MultipleLocator
from robot import Robot
from ball import Ball
from strategy import DummyStrategy
import matplotlib.pyplot as plt

PHYSICS_UPDATE_INTERVAL = 0.1
STRATEGY_UPDATE_INTERVAL = 5 # Every 5 physics steps
DISPLAY_UPDATE_INTERVAL = 5 # Every 5 physics steps

def displayGameState(robots, ball, t=0.0):
    foreground = plt.gcf().axes[1]
    foreground.clear()
    foreground.axis('equal')
    foreground.set_xlim([-3.5, 3.5])
    foreground.set_ylim([-5, 5])

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
    x = ball.get_position()[0]
    y = ball.get_position()[1]
    dx = ball.get_velocity()[0]
    dy = ball.get_velocity()[1]
    foreground.add_patch(plt.Circle((x, y), 0.5 / 2 / math.pi, color='black'))
    foreground.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1)

    # GUI text
    foreground.text(-3, 4.5, "Time: {0:.6g}".format(t))

    plt.pause(0.001)

def updateEstimatedPhysics(robots, ball):
    # Robot
    for robot in robots:
        # TODO use the same trajectory as in soccer_pycontrol
        if robot.status == Robot.Status.WALKING:
            delta = (robot.goal_position - robot.get_position())[0:2]
            if np.linalg.norm(delta) == 0:
                continue

            unit = delta / np.linalg.norm(delta)
            robot.position[0:2] = robot.get_position()[0:2] + unit * robot.speed * PHYSICS_UPDATE_INTERVAL
        elif robot.status == Robot.Status.KICKING:
            if ball.kick_timeout == 0:
                ball.velocity = robot.kick_velocity
            robot.status = Robot.Status.READY

    # Ball
    if ball.kick_timeout > 0:
        ball.kick_timeout = ball.kick_timeout - 1
    ball.position = ball.get_position() + ball.get_velocity() * PHYSICS_UPDATE_INTERVAL
    ball.velocity = ball.velocity * Ball.FRICTION_COEFF

if __name__ == '__main__':
    # Rules and Dimensions https://cdn.robocup.org/hl/wp/2021/04/V-HL21_Rules_changesMarked.pdf
    robots = [
        Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
              position=np.array([0.0, -3.5, math.pi / 2])),
        Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
              position=np.array([-1.5, -1.5, math.pi / 2])),
        Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
              position=np.array([1.5, -1.5, math.pi / 2])),
        Robot(team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY,
              position=np.array([0.0, -0.8, math.pi / 2])),
        Robot(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
              position=np.array([0.0, 3.5, -math.pi / 2])),
        Robot(team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
              position=np.array([-1.5, 1.5, -math.pi / 2])),
        Robot(team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
              position=np.array([1.5, 1.5, -math.pi / 2])),
        Robot(team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY,
              position=np.array([0.0, 0.8, -math.pi / 2]))
    ]
    ball = Ball(position=np.array([0, 0]))

    robots_init = copy.deepcopy(robots)
    ball_init = copy.deepcopy(ball)

    fig = plt.figure(figsize=(6.0, 9.0), dpi=60)
    background = fig.add_axes([0, 0, 1, 1])
    background.axis('equal')
    background.set_xlim([-3.5, 3.5])
    background.set_ylim([-5, 5])
    background.xaxis.set_major_locator(MultipleLocator(1))
    background.yaxis.set_major_locator(MultipleLocator(1))
    background.xaxis.set_minor_locator(MultipleLocator(0.1))
    background.yaxis.set_minor_locator(MultipleLocator(0.1))
    background.grid(which='minor', alpha=0.2)
    background.grid(which='major', alpha=0.5)
    background.add_patch(plt.Rectangle((-3, -4.5), 6, 9, alpha=0.1, color='green'))
    background.add_patch(plt.Rectangle((-1.3, -4.55), 2.6, 0.05, color='blue'))
    background.add_patch(plt.Rectangle((-1.3, 4.5), 2.6, 0.05, color='blue'))
    background.add_line(plt.Line2D((-3, 3), (0, 0), color='blue'))
    background.add_patch(plt.Circle((-0, 0), 1.3/2, fill=None, color='blue'))
    foreground = fig.add_axes([0, 0, 1, 1])
    foreground.set_facecolor((0,0,0,0))

    # Setup the strategy
    strategy = DummyStrategy()

    game_period_steps = int(2 * 10 * 60 / PHYSICS_UPDATE_INTERVAL) # 2 Periods of 10 minutes each
    for step in range(game_period_steps):
        if step > game_period_steps / 2:
            print("Second Half Started: ")
            robots = robots_init
            ball = ball_init

        if step % STRATEGY_UPDATE_INTERVAL == 0:
            strategy.update_both_team_strategy(robots, ball)

        updateEstimatedPhysics(robots, ball)

        if step % DISPLAY_UPDATE_INTERVAL == 0:
            displayGameState(robots, ball, step * PHYSICS_UPDATE_INTERVAL)

    plt.show()

