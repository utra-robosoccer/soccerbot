classdef soccerbot_base < handle
    properties(Access = private)
        robot_data = coder.load('generated/robot.mat');
        pose = Geometry.transform;              % Pose of the center between the two legs position
        end_position;
    end
    
    methods
        function obj = soccerbot_base()
        end
        
        function footpath = getPath(obj, finishPosition)
            crotch = obj.pose.position;
            finishPositionCoordinate = finishPosition.position;
            finishPositionCoordinate(3) = crotch(3);
            finishPosition.setPosition(finishPositionCoordinate);
            
            footpath = Geometry.robotpath(obj.pose, finishPosition, ...
                obj.robot_data.robot.foot_center_to_floor);
        end
        
        function ik_right_foot(obj, torso_to_right_foot)
            
        end
        
        function ik_left_foot(obj, torso_to_left_foot)
            
        end
        
        function stepPath(obj, t, robot_path)
            crotch_position = robot_path.crotchPosition(t) * obj.torso_offset;
            [right_foot_position, left_foot_position] = robot_path.footPosition(t);
                        
            torso_to_left_foot = crotch_position \ left_foot_position;
            torso_to_right_foot = crotch_position \ right_foot_position;
            
            configSolR = obj.ik_right_foot(torso_to_right_foot);
            configSolL = obj.ik_left_foot(torso_to_left_foot);
            
            obj.configuration(5:10) = configSolL;
            obj.configuration(13:18) = configSolR;
            
            obj.pose = Geometry.transform(crotch_position);
        end
        
        function config = configuration(obj)
            config = zeros(18,1);
        end
    end
    
    methods(Static)
        function generate()
            robotParameters;
            foot_center_to_floor = -right_collision_center(3) + foot_box(3);
            robot = Robot.soccerbot([0, 0, hip_height], foot_center_to_floor);
            robot = Robot.soccerbot_base.obj2struct(robot);
            save 'generated/robot.mat' robot;
        end
        
        function output_struct = obj2struct(o)
            % Converts obj into a struct by examining the public properties of obj. If
            % a property contains another object, this function recursively calls
            % itself on that object. Else, it copies the property and its value to 
            % output_struct. This function treats structs the same as objects.
            
            if isobject(o)
                o = struct(o);
            end
            
            if isstruct(o)
                
                properties = fieldnames(o);
                for i = 1:length(properties)
                    if (strcmp(properties{i}, 'Parent') || strcmp(properties{i}, 'Children'))
                        o.(properties{i}) = 0;
                    end
                    
                    if (strcmp(properties{i}, 'configuration') || strcmp(properties{i}, 'JointPosition'))
                        o.(properties{i}) = [o.configuration.JointPosition];
                        continue
                    end

                    o.(properties{i}) = Robot.soccerbot_base.obj2struct(o.(properties{i}));
                end
            end
            
            if iscell(o)
                for i = 1:length(o)
                    o{i} = Robot.soccerbot_base.obj2struct(o{i});
                end
            end
            
            output_struct = o;
        end
    end
end

