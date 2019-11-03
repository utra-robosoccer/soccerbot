%% Initialize the robot
robotParameters;
robot = Robot.soccerbot([-0.5, 0, hip_height]);
robot.show();

%% Create Robot Path
hold on;
start_position = Geometry.transform([-0.5 0 hip_height]);
end_position = Geometry.transform([0.5 0.5 hip_height]);
robot_path = Geometry.footpath(start_position, end_position);
robot_path.show;

