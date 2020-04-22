clear; close all;

robot = Robot.soccerbot_runner(1);
loop = 0;
simulink = 0;

if exist('run_from_bash', 'var')
    robot_name = "";
else
    robot_name = "robot1/";
end

robot.initialize_connections("robot1/");

if loop
    robot.loop();
elseif simulink
    robot.find_path([0.3 0.0], 0);
    robot.calculate_angles();
    robot.simulate();
else
    robot.find_path([0.3 0.0], 0);
    robot.calculate_angles();
    robot.setup_movement();
    robot.run_trajectory();
end

