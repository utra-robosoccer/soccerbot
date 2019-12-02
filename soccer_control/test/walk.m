%% Initialize the robot
robotParameters;
foot_center_to_floor = -right_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([-0.5, 0, hip_height], foot_center_to_floor);
start_position = robot.pose.position();
start_configuration = robot.configuration;
robot.show();

%% Create Robot Path
hold on;

robot.pose = Geometry.transform(start_position);
end_position = Geometry.transform([0.0 0 0]);
robot_path = robot.getPath(end_position);
robot_path.show();
figure;
robot_path.showTimingDiagram();

%% Calculate robot angles

angles = timeseries;
for t = 0:robot_path.step_size:robot_path.duration
    robot.stepPath(t, robot_path);
    angles = angles.addsample('Time',t, 'Data',robot.getAngles);
end

%% Follow the path (simulink)

load_system('soccerbot');
in = Simulink.SimulationInput('soccerbot');
in = in.setModelParameter('StartTime', '0', 'StopTime', num2str(angles.TimeInfo.End));
in = in.setModelParameter('SimulationMode', 'Normal');

in = in.setExternalInput('angles');
in = in.setVariable('angles',angles);

simOut = sim(in);
