classdef soccer_system < matlab.System

    properties
    end

    properties(DiscreteState)

    end

    properties(Access = private)
        foot_center_to_floor = 0.0221; % Generated from robotParameters
        hip_height = 0.18;             % TODO: move into soccerbot
        robot;
    end

    methods(Access = protected)
        function setupImpl(obj)
             obj.robot = Robot.soccerbot_base();
%             obj.robot.initialize([-0.5, 0, obj.hip_height], obj.foot_center_to_floor);
%             obj.start_position = obj.robot.pose.position();
%             obj.end_position = Geometry.transform([0 0 0]);
%             obj.robot_path = obj.robot.getPath(end_position);
        end

        function y = stepImpl(obj,u)
            % Implement algorithm. Calculate y as a function of input u and
            % discrete states.
            y = zeros(18,1);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function num = getNumOutputsImpl(~)
            num = 1;
        end

        function [sz1] = getOutputSizeImpl(obj)
            sz1 = 18;
        end

        function [fz1] = isOutputFixedSizeImpl(~)
            fz1 = true;
        end

        function [dt1] = getOutputDataTypeImpl(obj)
            dt1 = 'double';
        end

        function [cp1] = isOutputComplexImpl(obj)
            cp1 = false;
        end
    end
end
