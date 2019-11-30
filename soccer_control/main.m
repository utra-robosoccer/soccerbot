% Setup the publishers and subscribers
close all; clear; clc;
rosshutdown;
rosinit('192.168.0.120', 'NodeName', '/soccer_control');
motor_list = rosparam("get", "soccer_hardware/motor_mapping");

motors = fieldnames(motor_list);
pubs = containers.Map;
subs = containers.Map;
for i = 1:numel(motors)
    pubs(motors{i}) = rospublisher(strcat(motors{i}, "/command"), "std_msgs/Float64");
    subs(motors{i}) = rossubscriber(strcat(motors{i}, "/state"), "control_msgs/JointControllerState");
end
imu_sub = rossubscriber("imu", "sensor_msgs/Imu");

robotParameters;
foot_center_to_floor = -right_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([-0.5, 0, hip_height], foot_center_to_floor);

start_position = robot.pose.position();
end_position = Geometry.transform([0 0 0]);

% Create path of the robot
robot_path = robot.getPath(end_position);
% robot.show();
% hold on;
% robot_path.show();
% figure;
% robot_path.showTimingDiagram();
% 
% figure;
% Create the path of the robot into a timeseries
rate = rateControl(1/robot_path.step_size);
for t = 0:robot_path.step_size:robot_path.duration
    for i = 1:numel(robot.configuration)
        msg = pubs(robot.configuration(i).JointName).rosmessage;
        msg.Data = robot.configuration(i).JointPosition;
        p = pubs(robot.configuration(i).JointName);
        p.send(msg);
    end
%     waitfor(rate);
%     robot.show();
%     view(-90,0);
%     campos([-6,0,0]);
    
    robot.stepPath(t, robot_path);
end
