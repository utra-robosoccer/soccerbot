import numpy as np
import rospy
from matplotlib import pyplot as plt
from robot_ros import RobotRos
import game_engine


class GameEngineRos(game_engine.GameEngine):

    def __init__(self):
        # Listen to rostopics and get robots in field
        # setup subscribers to robot and ball positions in ros
        #
        pass

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
        # Robot
        for robot in robots:
            # TODO use the same trajectory as in soccer_pycontrol
            if robot.status == RobotRos.Status.WALKING:
                # publish a goal robot.goal_position geometry_msgs/Pose2D to /robot_name/goal
                i = 0
            elif robot.status == RobotRos.Status.KICKING:
                # publish a static trajectory
                pass


    def resetRobots(self):
        # Call the webots simulator to reset robot positions
        pass