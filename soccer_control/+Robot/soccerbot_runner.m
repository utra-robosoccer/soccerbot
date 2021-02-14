classdef soccerbot_runner < Robot.soccerbot
    properties
        debug = 0;
        enable_feedback = 0;
        hip_height;
        robot_path;
        angles;
        
        imu;
        motors;
        rate;

        pubs;
        subs;
        imu_sub;
        odom_pub;
        goal_sub;
        pose_sub;
    end
    
    methods
        function obj = soccerbot_runner(debug)
            
            robotParameters;
            foot_center_to_floor = -right_collision_center(3) + foot_box(3);
            
            obj = obj@Robot.soccerbot([0, 0, hip_height], foot_center_to_floor);
            obj.debug = debug;
            obj.hip_height = hip_height;
        end
        
        function initialize_connections(obj, robot_name)
            if nargin == 1
                robot_name = "";
            end
            
            rosshutdown;
            rosinit('localhost', 'NodeName', 'soccer_control');
            motor_list = rosparam("get", strcat(robot_name, "motor_mapping"));

            obj.motors = fieldnames(motor_list);
            obj.pubs = containers.Map;
            obj.subs = containers.Map;
            for i = 1:numel(obj.motors)
                obj.pubs(obj.motors{i}) = rospublisher(strcat(robot_name, obj.motors{i}, "/command"), "std_msgs/Float64");
            %     obj.subs(motors{i}) = rossubscriber(strcat(robot_name, obj.motors{i}, "/state"), "control_msgs/JointControllerState");
            end


            obj.goal_sub = rossubscriber(strcat(robot_name, "goal"), "geometry_msgs/PoseStamped");
            obj.pose_sub = rossubscriber(strcat(robot_name, "amcl_pose"), "geometry_msgs/PoseWithCovarianceStamped", @obj.pose_callback);
            obj.imu_sub = rossubscriber(strcat(robot_name, "imu"), "sensor_msgs/Imu", @obj.imu_callback);
            obj.odom_pub = rospublisher(strcat(robot_name, "odom"), "nav_msgs/Odometry");
        end
        
        function find_path(obj, dest, angle)
            end_position = Geometry.transform([dest obj.hip_height], eul2quat([0 0 angle]));
            obj.robot_path = obj.getPath(end_position);
            obj.robot_path.show();
            figure;
            obj.robot_path.showTimingDiagram();
            obj.rate = rateControl(1/obj.robot_path.step_size);
        end
        
        function calculate_angles(obj)
            obj.angles = timeseries;
            for t = 0:obj.robot_path.step_size:obj.robot_path.duration
                obj.stepPath(t, obj.robot_path);
                obj.angles = obj.angles.addsample('Time',t, 'Data',obj.getAngles);
            end
            figure();
            subplot(3,1,1);
            plot(obj.angles.Time, obj.angles.Data(:,5:10));
            title('One Foot Motor Angles');
            xlabel('time (t)');
            ylabel('Angles');
            grid off;
            grid minor;
            legend('M1', 'M2', 'M3', 'M4', 'M5', 'M6');
            
            subplot(3,1,2);
            plot(obj.angles.Time, obj.angles.Data(:,13:18));
            title('Another Foot Motor Angles');
            xlabel('time (t)');
            ylabel('Angles');
            grid off;
            grid minor;
            legend('M1', 'M2', 'M3', 'M4', 'M5', 'M6');
            
            subplot(3,1,3);
            plot(obj.angles.Time, obj.angles.Data(:,1:4));
            hold on;
            plot(obj.angles.Time, obj.angles.Data(:,11:12));
            hold off;
            title('Head & Arms Motor Angles');
            xlabel('time (t)');
            ylabel('Angles');
            grid off;
            grid minor;
            legend('M1', 'M2', 'M3', 'M4', 'M5', 'M6');
            % legend('Left1 (Hip)', 'Left2', 'Left3');
            
        end
        
        function loop(obj)
            while 1
                disp("Waiting for Goal");
                goal = obj.goal_sub.receive();
                disp("Recieved Goal, executing walk");
                
                angle = quat2eul([pose.Pose.Pose.Orientation.W pose.Pose.Pose.Orientation.X pose.Pose.Pose.Orientation.Y pose.Pose.Pose.Orientation.Z]);
                robot.updatePosition([pose.Pose.Pose.Position.X pose.Pose.Pose.Position.Y obj.hip_height], eul2quat([0, 0, angle(3)]));
                angle = quat2eul([goal.Pose.Orientation.W goal.Pose.Orientation.X goal.Pose.Orientation.Y goal.Pose.Orientation.Z]);

                % Create path of the robot
                obj.find_path([goal.Pose.Position.X goal.Pose.Position.Y], angle(3));
                
                % Calculate angles
                obj.calculate_angles();

                % Get ready to move
                obj.setup_movement();
                
                % Run the trajectory
                obj.run_trajectory();
            end
        end
        
        function setup_movement(obj)
            for t = 0:obj.robot_path.step_size:3
                for i = 1:numel(obj.configuration)
                    msg = obj.pubs(obj.configuration(i).JointName).rosmessage;
                    msg.Data = obj.configuration(i).JointPosition;
                    if contains(obj.configuration(i).JointName, "head")
                        continue
                    end

                    p = obj.pubs(obj.configuration(i).JointName);
                    p.send(msg);
                end
                waitfor(obj.rate);
            end
            obj.imu = obj.imu_sub.receive();
            angle = [obj.imu.Orientation.W obj.imu.Orientation.X obj.imu.Orientation.Y obj.imu.Orientation.Z];
            obj.rpy_current = quat2eul(angle);
        end
        
        function run_trajectory(obj)

            for t = 0:obj.robot_path.step_size:obj.robot_path.duration
                for i = 1:numel(obj.configuration)
                    msg = obj.pubs(obj.configuration(i).JointName).rosmessage;
                    msg.Data = obj.configuration(i).JointPosition;
                    if contains(obj.configuration(i).JointName, "head")
                        continue
                    end

                    p = obj.pubs(obj.configuration(i).JointName);
                    p.send(msg);
                end
                
                if (obj.enable_feedback)
                    try
                        angle = [obj.imu.Orientation.W obj.imu.Orientation.X obj.imu.Orientation.Y obj.imu.Orientation.Z];
                        obj.applyRPYFeedback(quat2eul(angle));
                    catch ex
                    end                    
                end

                % Publish odom
                robot_position = obj.pose.position;
                robot_orientation = obj.pose.orientation;
                msg = obj.odom_pub.rosmessage;
                msg.Header.Stamp = obj.imu.Header.Stamp;
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

                obj.odom_pub.send(msg);

                % Step Path
                obj.stepPath(t, obj.robot_path);

                % Wait
                waitfor(obj.rate);

                % Debug
            %     robot.show();
            %     view(-90,0);
            %     campos([-6,0,0]);
            end
        end
        
        function imu_callback(obj, ~, imu_data)
            obj.imu = imu_data;
        end
        
        function pose_callback(obj, ~, pose_data)
            obj.pose = pose_data;
        end
        
        function simOut = simulate(obj)
            load_system('soccerbot');
            in = Simulink.SimulationInput('soccerbot');
            in = in.setModelParameter('StartTime', '0', 'StopTime', num2str(obj.angles.TimeInfo.End));
            in = in.setModelParameter('SimulationMode', 'Normal');

            in = in.setExternalInput('angles');
            in = in.setVariable('angles',obj.angles);

            simOut = sim(in);
        end
    end
end

