import numpy as np
import rospy
from matplotlib import pyplot as plt
from robot_ros import RobotRos
from ball import Ball
import game_engine
import copy
import geometry_msgs.msg
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from strategy import DummyStrategyROS

class GameEngineRos(game_engine.GameEngine):
    KICK_TIMEOUT = 5
    GETUPFRONT_TIMEOUT = 10
    GETUPBACK_TIMEOUT = 10

    def __init__(self):
        # Listen to rostopics and get robots in field
        # setup subscribers to robot and ball positions in ros
        self.robots = [
            RobotRos(team=RobotRos.Team.FRIENDLY, role=RobotRos.Role.GOALIE, status=RobotRos.Status.READY,
                     robot_name="robot1"),
            RobotRos(team=RobotRos.Team.FRIENDLY, role=RobotRos.Role.LEFT_MIDFIELD, status=RobotRos.Status.READY,
                     robot_name="robot2"),
            RobotRos(team=RobotRos.Team.FRIENDLY, role=RobotRos.Role.RIGHT_MIDFIELD, status=RobotRos.Status.READY,
                     robot_name="robot3"),
            RobotRos(team=RobotRos.Team.FRIENDLY, role=RobotRos.Role.STRIKER, status=RobotRos.Status.READY,
                     robot_name="robot4"),
            RobotRos(team=RobotRos.Team.OPPONENT, role=RobotRos.Role.GOALIE, status=RobotRos.Status.READY,
                     robot_name="robot5"),
            RobotRos(team=RobotRos.Team.OPPONENT, role=RobotRos.Role.LEFT_MIDFIELD, status=RobotRos.Status.READY,
                     robot_name="robot6"),
            RobotRos(team=RobotRos.Team.OPPONENT, role=RobotRos.Role.RIGHT_MIDFIELD, status=RobotRos.Status.READY,
                     robot_name="robot7"),
            RobotRos(team=RobotRos.Team.OPPONENT, role=RobotRos.Role.STRIKER, status=RobotRos.Status.READY,
                     robot_name="robot8"),
        ]

        self.ball = Ball(position=np.array([0, 0]))

        #self.robots_init = copy.deepcopy(self.robots)
        #self.ball_init = copy.deepcopy(self.ball)

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
        background.add_patch(plt.Circle((-0, 0), 1.3 / 2, fill=None, color='blue'))
        foreground = fig.add_axes([0, 0, 1, 1])
        foreground.set_facecolor((0, 0, 0, 0))

        # Setup the strategy
        self.strategy = DummyStrategyROS()

    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future
        ball_positions = np.array([])
        for robot in self.robots:
            if not np.isnan(robot.ball_position.any()):
                np.append(ball_positions, robot.ball_position, axis=0)
        if ball_positions.size != 0:
            self.ball.position = ball_positions.mean(axis=0)

    def run(self):
        game_period_steps = int(2 * 10 * 60 / GameEngineRos.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 10 minutes each

        friendly_points = 0
        opponent_points = 0

        half_time_started = False

        rostime_previous = 0
        while not rospy.is_shutdown():
            rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

            if rostime > 10 and not half_time_started:
                half_time_started = True
                print("Second Half Started: ")
                self.resetRobots()

            if rostime % (GameEngineRos.STRATEGY_UPDATE_INTERVAL * GameEngineRos.PHYSICS_UPDATE_INTERVAL) < \
                    rostime_previous % (GameEngineRos.STRATEGY_UPDATE_INTERVAL * GameEngineRos.PHYSICS_UPDATE_INTERVAL):
                self.strategy.update_both_team_strategy(self.robots, self.ball)

            self.updateEstimatedPhysics(self.robots, self.ball)

            # Check victory condition
            if self.ball.get_position()[1] > 4.5:
                print("Friendly Scores!")
                friendly_points += 1
                self.robots = self.robots_init
                self.ball = self.ball_init
            elif self.ball.get_position()[1] < -4.5:
                print("Opponent Scores!")
                opponent_points += 1
                robots = self.robots_init
                self.ball = self.ball_init

            if rostime % (GameEngineRos.DISPLAY_UPDATE_INTERVAL * GameEngineRos.PHYSICS_UPDATE_INTERVAL) < \
                    rostime_previous % (GameEngineRos.DISPLAY_UPDATE_INTERVAL * GameEngineRos.PHYSICS_UPDATE_INTERVAL):
                self.displayGameState(self.robots, self.ball, rostime)

            rostime_previous = rostime

        print(F"Game Finished: Friendly: {friendly_points}, Opponent: {opponent_points}")
        plt.show()

    def updateEstimatedPhysics(self, robots, ball):
        rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9
        for robot in robots:
            if robot.status == RobotRos.Status.WALKING:
                # publish a goal robot.goal_position geometry_msgs/Pose2D to /robot_name/goal
                goal_msg = geometry_msgs.msg.Pose2D()
                goal_msg.x = robot.goal_position[0]
                goal_msg.y = robot.goal_position[1]
                goal_msg.theta = robot.goal_position[2]
                robot.pub_goal.publish(goal_msg)
            elif robot.status == RobotRos.Status.KICKING:
                # if kick timout is not active
                if rostime - robot.last_kick > self.KICK_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = RobotRos.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.pub_trajectory.publish("data: rightkick")
                        robot.last_kick = rostime
                        robot.publishing_static_trajectory = True

            elif robot.status == RobotRos.Status.FALLEN_BACK:
                # if timout is not active
                if rostime - robot.last_getupback > self.GETUPBACK_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = RobotRos.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.pub_trajectory.publish("data: getupback")
                        robot.last_getupback = rostime
                        robot.publishing_static_trajectory = True

            elif robot.status == RobotRos.Status.FALLEN_FRONT:
                # if timout is not active
                if rostime - robot.last_getupfront > self.GETUPFRONT_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = RobotRos.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.pub_trajectory.publish("data: getupfront")
                        robot.last_getupfront = rostime
                        robot.publishing_static_trajectory = True

        self.update_average_ball_position()

    def resetRobots(self):
        # Call the webots simulator to reset robot positions
        pass
