%% Initialize the robot
robotParameters;
robot = Robot.soccerbot([-0.5, 0, hip_height + foot_box(3)]);
robot.show();

%% Create Robot Path
hold on;

step_size = 0.01;

start_position = Geometry.transform([-0.5 0 hip_height + foot_box(3)]);
end_position = Geometry.transform([0.5 0.5 hip_height + foot_box(3)]);
robot_path = Geometry.crotchpath(start_position, end_position);
robot_path.show();

hold on
start_position = Geometry.transform([-0.5 0 foot_box(3)]);
end_position = Geometry.transform([0.5 0.5 foot_box(3)]);
robot_path = Geometry.footpath(start_position, end_position);
robot_path.show();

