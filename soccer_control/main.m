% Setup the publishers and subscribers
close all; clear; clc;
rosshutdown;
rosinit('192.168.0.120', 'NodeName', 'robot1/soccer_control');
motor_list = rosparam("get", "robot1/soccer_hardware/motor_mapping");

motors = fieldnames(motor_list);
pubs = containers.Map;
subs = containers.Map;
for i = 1:numel(motors)
    pubs(motors{i}) = rospublisher(strcat("robot1/", motors{i}, "/command"), "std_msgs/Float64");
%     subs(motors{i}) = rossubscriber(strcat("robot1/", motors{i}, "/state"), "control_msgs/JointControllerState");
end

imu_sub = rossubscriber("robot1/imu", "sensor_msgs/Imu");
odom_pub = rospublisher("robot1/odom", "nav_msgs/Odometry");
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
for t = 0:robot_path.step_size:10000%robot_path.duration
    for i = 1:numel(robot.configuration)
        msg = pubs(robot.configuration(i).JointName).rosmessage;
        msg.Data = robot.configuration(i).JointPosition;
        if contains(robot.configuration(i).JointName, "head")
            continue
        end
        
        p = pubs(robot.configuration(i).JointName);
        p.send(msg);
    end
%     waitfor(rate);
%     robot.show();
%     view(-90,0);
%     campos([-6,0,0]);
    
    robot_position = robot.pose.position;
    robot_orientation = robot.pose.orientation;
    msg = odom_pub.rosmessage;
    msg.Header.Stamp = rostime('now');
    msg.Header.FrameId = "odom";
    msg.ChildFrameId = "base_footprint";
    msg.Pose.Pose.Position.X = robot_position(1);
    msg.Pose.Pose.Position.Y = robot_position(2);
    msg.Pose.Pose.Position.Z = robot_position(3);
    
    msg.Pose.Pose.Orientation.W = robot_orientation(1);
    msg.Pose.Pose.Orientation.X = robot_orientation(2);
    msg.Pose.Pose.Orientation.Y = robot_orientation(3);
    msg.Pose.Pose.Orientation.Z = robot_orientation(4);
    msg.Pose.Covariance(6 * 0 + 1) = 0.01;
    msg.Pose.Covariance(6 * 1 + 2) = 0.01;
    msg.Pose.Covariance(6 * 2 + 3) = 0.01;
    msg.Pose.Covariance(6 * 3 + 4) = 0.01;
    msg.Pose.Covariance(6 * 4 + 5) = 0.01;
    msg.Pose.Covariance(6 * 5 + 6) = 0.01;
    
    odom_pub.send(msg);
    
%    robot.stepPath(t, robot_path);
end