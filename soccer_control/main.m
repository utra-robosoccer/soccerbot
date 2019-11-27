% Setup the publishers and subscribers
rosshutdown;
rosinit('NodeName', '/soccer_control');
motor_list = rosparam("get", "motor_mapping");

motors = fieldnames(motor_list);
pubs = containers.Map;
subs = containers.Map;
for i = 1:numel(motors)
    pubs(motors{i}) = rospublisher(strcat(motors{i}, "/command"), "std_msgs/Float64");
    subs(motors{i}) = rossubscriber(strcat(motors{i}, "/state"), "control_msgs/JointControllerState");
end
imu_sub = rossubscriber("imu", "sensor_msgs/Imu");

robotParameters;
foot_center_to_floor = -left_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([-0.5, 0, hip_height], foot_center_to_floor);

start_position = robot.pose.position();
end_position = Geometry.transform([0 0 0]);

% Create path of the robot
robot_path = robot.getPath(end_position);
robot_path.show();
robot_path.showTimingDiagram();

% Create the path of the robot into a timeseries
rate = rateControl(1/robot_path.step_size);
for t = 0:robot_path.step_size:robot_path.duration
    robot.stepPath(t, robot_path);
    for i = 1:numel(robot.configuration)
        msg = pubs(robot.configuration(i).JointName).rosmessage;
        msg.Data = robot.configuration(i).JointPosition;
        p = pubs(robot.configuration(i).JointName);
        p.send(msg);
    end
    waitfor(rate);
end
