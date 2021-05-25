import numpy as np

import rospy
import std_msgs.msg

import game_engine
from robot import Robot
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator
from strategy import DummyStrategy
from robot_ros import RobotRos
from ball import Ball


class GameEngineRos(game_engine.GameEngine):
    KICK_TIMEOUT = 3
    GETUPFRONT_TIMEOUT = 7
    GETUPBACK_TIMEOUT = 10

    def __init__(self, display=True):
        # Listen to rostopics and get robots in field
        # setup subscribers to robot and ball positions in ros
        '''self.robots = [
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                     robot_name="robot1"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                     robot_name="robot2"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                     robot_name="robot3"),
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                     robot_name="robot4"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                     robot_name="robot5"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.LEFT_MIDFIELD, status=Robot.Status.READY,
                     robot_name="robot6"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.RIGHT_MIDFIELD, status=Robot.Status.READY,
                     robot_name="robot7"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.STRIKER, status=Robot.Status.READY,
                     robot_name="robot8"),
        ]'''

        self.robots = [
            RobotRos(team=Robot.Team.FRIENDLY, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                     robot_name="robot1"),
            RobotRos(team=Robot.Team.OPPONENT, role=Robot.Role.GOALIE, status=Robot.Status.READY,
                     robot_name="robot2")
        ]
        self.ball = Ball(position=np.array([0, 0]))
        self.last_ball_pose = self.ball.get_position()

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

        self.reset_pub = rospy.Publisher('/reset', std_msgs.msg.String, queue_size=1)

        # Setup the strategy
        self.strategy = DummyStrategy()

    def update_average_ball_position(self):
        # get estimated ball position with tf information from 4 robots and average them
        # this needs to be team-dependent in the future
        ball_positions = []
        for robot in self.robots:
            if robot.ball_position.all():
                ball_positions.append(robot.ball_position)

        if ball_positions:
            self.ball.position = np.array(ball_positions).mean(axis=0)

    def run(self):
        game_period_steps = int(2 * 10 * 60 / GameEngineRos.PHYSICS_UPDATE_INTERVAL)  # 2 Periods of 10 minutes each

        friendly_points = 0
        opponent_points = 0

        half_time_started = True

        rostime_previous = 0
        rostime_initial = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9
        while not rospy.is_shutdown():
            rostime = rospy.get_rostime().secs + rospy.get_rostime().nsecs * 1e-9

            if rostime > rostime_initial + 10 and not half_time_started:
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
        #print(str(robots[0].status) + " " + str(robots[1].status) + " " + str(robots[2].status) + " " + str(robots[3].status))
        for robot in robots:
            if robot.status == Robot.Status.WALKING:
                # publish a goal robot.goal_position geometry_msgs/Pose2D to /robot_name/goal
                pass
            elif robot.status == Robot.Status.KICKING:
                # if kick timout is not active
                if rostime - robot.last_kick > self.KICK_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = Robot.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.trajectory_publisher.publish("rightkick")
                        robot.last_kick = rostime
                        robot.publishing_static_trajectory = True
                        if robot.robot_name == "robot1":
                            print("kicking")

            elif robot.status == Robot.Status.FALLEN_BACK:
                # if timout is not active
                if rostime - robot.last_getupback > self.GETUPBACK_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = Robot.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.terminate_walking_publisher.publish()
                        robot.trajectory_publisher.publish("getupback")
                        robot.last_getupback = rostime
                        robot.publishing_static_trajectory = True
                        if robot.robot_name == "robot1":
                            print("getupback")

            elif robot.status == Robot.Status.FALLEN_FRONT:
                # if timout is not active
                if rostime - robot.last_getupfront > self.GETUPFRONT_TIMEOUT:
                    # if finished publishing trajectory, reset status to READY
                    if robot.publishing_static_trajectory:
                        robot.publishing_static_trajectory = False
                        robot.status = Robot.Status.READY
                    # else, publish trajectory, update timeout
                    else:
                        robot.terminate_walking_publisher.publish()
                        robot.trajectory_publisher.publish("getupfront")
                        robot.last_getupfront = rostime
                        robot.publishing_static_trajectory = True
                        if robot.robot_name == "robot1":
                            print("getupfront")

            if robot.status != robot.previous_status and robot.robot_name == "robot1":
                print(robot.robot_name + " status changes to " + str(robot.status))
                robot.previous_status = robot.status

        self.update_average_ball_position()

    def resetRobots(self):
        # Call the webots simulator to reset robot positions
        self.reset_pub.publish("reset")
        pass
