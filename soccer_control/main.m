%% Initialize the robot
robotParameters;
foot_center_to_floor = -left_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([-0.5, 0, hip_height], foot_center_to_floor);
robot.show();

%% Create Robot Path
hold on;

end_position = Geometry.transform([0.5 0.5 0]);
robot_path = robot.getPath(end_position);
robot_path.show();
figure;
robot_path.showTimingDiagram();

%% Follow the path

figure;
set(gca, 'CameraPosition', [16.2252546176456,1.97312476814183,5.73130798700572]);
for t = 0:robot_path.step_size:robot_path.duration
    robot.stepPath(t, robot_path);
    robot.show();
    view(110,20);
    zoom(1.5);
    drawnow;
end