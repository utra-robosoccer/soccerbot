from robot import Robot


class StopResumeRobots():
    def stop_all_robots(self, robots):
        for robot in robots:
            robot.stop_requested = True

    def resume_all_robots(self, robots):
        for robot in robots:
            robot.completed_trajectory_publisher.publish(True)
            if robot.stop_requested:
                robot.stop_requested = False
            if robot.status == Robot.Status.STOPPED:
                robot.status = Robot.Status.READY
