classdef soccerbot_base < handle
    properties
    end
    
    methods
        function obj = soccerbot_base()
            robot = coder.load('generated/robot.mat');
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

