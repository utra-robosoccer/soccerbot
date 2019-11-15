classdef soccerbot < handle
    properties
        robot = importrobot('../soccer_description/models/soccerbot_stl.urdf');
        robot_left_leg_subtree;
        robot_right_leg_subtree;
        configuration;
        pose = Geometry.transform;              % Pose of the center between the two legs position
        ik_left
        ik_right
        dh
    end
    
    methods
        function obj = soccerbot(position)
            obj.configuration = obj.robot.homeConfiguration;
            obj.robot_left_leg_subtree = obj.robot.subtree('left_hip_side');
            obj.robot_right_leg_subtree = obj.robot.subtree('right_hip_side');
            
            obj.ik_left = inverseKinematics('RigidBodyTree', obj.robot_left_leg_subtree);
            obj.ik_right = inverseKinematics('RigidBodyTree', obj.robot_right_leg_subtree);
            
            % Solve DH Table
            solveDHTable(obj);

            % Calculating hip and feet location
            hip_to_torso = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_hip_front');
            position(3) = position(3) + hip_to_torso(3,4);
            obj.pose.setPosition(position);
            
            foot_center_z_to_foot = 0.00737 + 0.014747; % Using foot box height and foot to thing parameter
            
            left_foot_position = obj.robot_left_leg_subtree.getTransform(obj.robot_left_leg_subtree.homeConfiguration, 'left_foot', 'torso');
            left_foot_position(3,4) = - position(3) + foot_center_z_to_foot;
            configSolL = obj.ik_left_foot(left_foot_position);
%             weights_left = [0.25 0.25 0.25 1 1 1];
%             [configSolL,~] = obj.ik_left('left_foot',left_foot_position,weights_left,obj.robot_left_leg_subtree.homeConfiguration);
            
            right_foot_position = obj.robot_right_leg_subtree.getTransform(obj.robot_right_leg_subtree.homeConfiguration, 'right_foot', 'torso');
            right_foot_position(3,4) = - position(3) + foot_center_z_to_foot;
            configSolR = obj.ik_right_foot(right_foot_position);
%             weights_right = [0.25 0.25 0.25 1 1 1];
%             [configSolR,~] = obj.ik_right('right_foot',right_foot_position,weights_right,obj.robot_right_leg_subtree.homeConfiguration);

            
            obj.configuration(3:8) = configSolL;
            obj.configuration(13:18) = configSolR;
            
            showdetails(obj.robot)
        end
        
        function footpath = getFootPath(obj, startPosition, finishPosition)
            
        end
        
        function solveDHTable(obj)
            H3 = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_thigh', 'left_calve');
            H4 = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_calve', 'left_ankle');
            
            syms theta1 theta2 theta3 theta4 theta5 theta6
            obj.dh = [0 -pi/2 0 theta1; ...
                  0 pi/2 0 theta2; ...
                  H3(3,4) 0 0 theta3; ...
                  H4(3,4) 0 0 theta4; ...
                  0 pi/2 0 theta5; ...
                  0 0 0 theta6];
        end
        
        function H = leftFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_foot');
        end
        
        function H = rightFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'right_foot');
        end
        
        function configSolL = ik_left_foot(obj, configuration)
            torso_to_hip = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'torso');
            configSolL = obj.robot_left_leg_subtree.homeConfiguration;
            
            configuration(1:3,4) = configuration(1:3,4) - torso_to_hip(1:3,4);
            invconf = inv(configuration);
            
            d3 = double(obj.dh(3,1));
            d4 = double(obj.dh(4,1));
            
            Xd = invconf(1,4);
            Yd = invconf(2,4);
            Zd = invconf(3,4);
            
            theta6 = atan2(Yd, Zd);
            
            tmp1 = Zd / cos(theta6);
            tmp2 = sqrt(Xd^2 + Yd^2);
            D = (tmp1^2 + tmp2^2 - d3^2 - d4^2) / 2 / d3 / d4;
            tmp3 = atan2(D, sqrt(1-D^2));
            theta4 = tmp3 - pi/2;
            
            alp = atan2(tmp1, tmp2);
            beta = atan2(-d3 * cos(tmp3), d4 + d3 * sin(tmp3));
            theta5 = alp - beta - pi/2;
            
            H4 = Geometry.transform(obj.dh(4,1), obj.dh(4,2), obj.dh(4,3), theta4);
            H5 = Geometry.transform(obj.dh(5,1), obj.dh(5,2), obj.dh(5,3), theta5);
            H6 = Geometry.transform(obj.dh(6,1), obj.dh(6,2), obj.dh(6,3), theta6);
            
            H46 = double(H4.H * H5.H * H6.H);
            
            H03 = configuration / H46;
            
            theta1 = atan2(H03(2,3), H03(1,3)) - pi/2;
            theta2 = atan2(sqrt(1-H03(3,3)^2), H03(3,3)) - pi/2;
            theta3 = atan2(-H03(3,2), H03(3,1)) - pi/2;
            
            configSolL(1).JointPosition = theta1;
            configSolL(2).JointPosition = theta2;
            configSolL(3).JointPosition = theta3;
            configSolL(4).JointPosition = theta4;
            configSolL(5).JointPosition = theta5;
            configSolL(6).JointPosition = theta6;
        end
        
        function configSolR = ik_right_foot(obj, configuration)
            left_hip_to_right_hip = obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'right_hip_front');

            configuration(1:3,4) = configuration(1:3,4) + left_hip_to_right_hip(1:3,4);
            configSolL = ik_left_foot(obj, configuration);
            
            configSolR = obj.robot_right_leg_subtree.homeConfiguration;
            configSolR(1).JointPosition = -configSolL(1).JointPosition;
            configSolR(2).JointPosition = -configSolL(2).JointPosition;
            configSolR(3).JointPosition = configSolL(3).JointPosition;
            configSolR(4).JointPosition = configSolL(4).JointPosition;
            configSolR(5).JointPosition = configSolL(5).JointPosition;
            configSolR(6).JointPosition = -configSolL(6).JointPosition;        
        end
        
        function show(obj)
            maprobot = robotics.RigidBodyTree;
            maplink = robotics.RigidBody('torso');
            maplink.Joint.setFixedTransform(obj.pose.H);
            maprobot.addBody(maplink, maprobot.BaseName);
            maprobot.addSubtree('torso', obj.robot);
            maprobot.show(obj.configuration);
            patch( [-3 -3 3 3 -3], [5 -5 -5 5 5], [0 0 0 0 0], 'green')

        end
    end
end

