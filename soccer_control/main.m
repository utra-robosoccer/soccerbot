%% Initialize the robot
robotParameters;
foot_center_to_floor = -left_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([-0.5, 0, hip_height], foot_center_to_floor);
robot.show();

%% Create Robot Path
hold on;

start_position = Geometry.transform([-0.5 0 0]);
end_position = Geometry.transform([0.5 0.5 0]);

robot_path = robot.getPath(start_position, end_position);
% robot_path.showTimingDiagram();

% robot_path.show();
