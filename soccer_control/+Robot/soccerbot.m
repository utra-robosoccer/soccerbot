classdef soccerbot
    properties
        robot = importrobot('../soccer_description/models/soccerbot_stl.urdf');
        robot_left_leg_subtree;
        robot_right_leg_subtree;
        configuration;
        pose = Geometry.transform;              % Pose of the center between the two legs position
        ik_left
        ik_right
    end
    
    methods
        function obj = soccerbot(position)
            obj.configuration = obj.robot.homeConfiguration;
            obj.robot_left_leg_subtree = obj.robot.subtree('left_hip_side');
            obj.robot_right_leg_subtree = obj.robot.subtree('right_hip_side');
            
            obj.ik_left = inverseKinematics('RigidBodyTree', obj.robot_left_leg_subtree);
            obj.ik_right = inverseKinematics('RigidBodyTree', obj.robot_right_leg_subtree);

            % Calculating hip and feet location
            hip_to_torso = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_hip_front');
            obj.pose.position = position;
            obj.pose.position(3) = obj.pose.position(3) + hip_to_torso(3,4);
            
            left_foot_position = obj.robot_left_leg_subtree.getTransform(obj.robot_left_leg_subtree.homeConfiguration, 'left_foot', 'torso');
            left_foot_position(3,4) = - obj.pose.position(3) + 0.01474;
            weights_left = [0.25 0.25 0.25 1 1 1];
            [configSolL,~] = obj.ik_left('left_foot',left_foot_position,weights_left,obj.robot_left_leg_subtree.homeConfiguration);
            
            right_foot_position = obj.robot_right_leg_subtree.getTransform(obj.robot_right_leg_subtree.homeConfiguration, 'right_foot', 'torso');
            right_foot_position(3,4) = - obj.pose.position(3) + 0.01474;
            weights_right = [0.25 0.25 0.25 1 1 1];
            [configSolR,~] = obj.ik_right('right_foot',right_foot_position,weights_right,obj.robot_right_leg_subtree.homeConfiguration);
            
            obj.configuration(3:8) = configSolL;
            obj.configuration(13:18) = configSolR;
            
            showdetails(obj.robot)
        end
        
        function H = leftFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_foot');
        end
        
        function H = rightFootPose(obj)
            H = obj.robot.getTransform(obj.robot.homeConfiguration, 'torso', 'left_foot');
        end
        
        function show(obj)
            maprobot = robotics.RigidBodyTree;
            maplink = robotics.RigidBody('torso');
            maplink.Joint.setFixedTransform(obj.pose.H);
            maprobot.addBody(maplink, maprobot.BaseName);
            maprobot.addSubtree('torso', obj.robot);
%             maprobot.getBody('torso').addVisual("Mesh",obj.robot.Base.Visuals{1});
            maprobot.show(obj.configuration);
            patch( [-3 -3 3 3 -3], [5 -5 -5 5 5], [0 0 0 0 0], 'green')

        end
    end
end

