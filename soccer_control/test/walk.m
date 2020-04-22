%% Initialize the robot
robot = Robot.soccerbot_runner(1);
robot.find_path([0.3 0.0], 0);
robot.calculate_angles();
robot.simulate();
