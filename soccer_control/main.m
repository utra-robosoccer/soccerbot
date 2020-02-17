if exist('run_from_bash', 'var')
    robot_name = "";
else
    close all; clear; clc;
    robot_name = "robot1/";
end

rosshutdown;
rosinit('localhost', 'NodeName', 'soccer_control');
motor_list = rosparam("get", strcat(robot_name, "motor_mapping"));

motors = fieldnames(motor_list);
pubs = containers.Map;
subs = containers.Map;
for i = 1:numel(motors)
    pubs(motors{i}) = rospublisher(strcat(robot_name, motors{i}, "/command"), "std_msgs/Float64");
%     subs(motors{i}) = rossubscriber(strcat(robot_name, motors{i}, "/state"), "control_msgs/JointControllerState");
end

global imu

goal_sub = rossubscriber(strcat(robot_name, "goal"), "geometry_msgs/PoseStamped");
pose_sub = rossubscriber(strcat(robot_name, "amcl_pose"), "geometry_msgs/PoseWithCovarianceStamped");
imu_sub = rossubscriber(strcat(robot_name, "imu"), "sensor_msgs/Imu", @imu_callback);
odom_pub = rospublisher(strcat(robot_name, "odom"), "nav_msgs/Odometry");

% Robot itself
robotParameters;
foot_center_to_floor = -right_collision_center(3) + foot_box(3);
robot = Robot.soccerbot([0.0, 0, hip_height], foot_center_to_floor);

while 1
    disp("Waiting for Goal");
    goal = goal_sub.receive();
    pose = pose_sub.receive();
    disp("Recieved Goal, executing walk");
    robot.updatePosition([pose.Pose.Pose.Position.X pose.Pose.Pose.Position.Y hip_height], ...
        [pose.Pose.Pose.Orientation.W pose.Pose.Pose.Orientation.X pose.Pose.Pose.Orientation.Y pose.Pose.Pose.Orientation.Z]);
    end_position = Geometry.transform([goal.Pose.Position.X goal.Pose.Position.Y hip_height], ...
        [goal.Pose.Orientation.W goal.Pose.Orientation.X goal.Pose.Orientation.Y goal.Pose.Orientation.Z]);
    
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

    imu = imu_sub.receive();
    rate = rateControl(1/robot_path.step_size);
    for t = 0:robot_path.step_size:robot_path.duration
        for i = 1:numel(robot.configuration)
            msg = pubs(robot.configuration(i).JointName).rosmessage;
            msg.Data = robot.configuration(i).JointPosition;
            if contains(robot.configuration(i).JointName, "head")
                continue
            end

            p = pubs(robot.configuration(i).JointName);
            p.send(msg);
        end

        try
            angle = [imu.Orientation.W imu.Orientation.X imu.Orientation.Y imu.Orientation.Z];
            robot.applyRPYFeedback(quat2eul(angle));
        catch ex
            disp "test";
        end

        % Publish odom
        robot_position = robot.pose.position;
        robot_orientation = robot.pose.orientation;
        msg = odom_pub.rosmessage;
        msg.Header.Stamp = imu.Header.Stamp;
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

        % Step Path
        robot.stepPath(t, robot_path);

        % Wait
        waitfor(rate);

        % Debug
    %     robot.show();
    %     view(-90,0);
    %     campos([-6,0,0]);
    end
end


function imu_callback(~, imu_data)
    global imu
    imu = imu_data;
end