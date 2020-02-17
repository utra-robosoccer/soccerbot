classdef soccer_system < matlab.System & matlab.system.mixin.SampleTime & matlab.system.mixin.Propagates

    properties
    end

    properties(DiscreteState)
        t;
        has_goal;
    end

    properties(Access = private)
        foot_center_to_floor = 0.0221; % Generated from robotParameters
        hip_height = 0.18;             % TODO: move into soccerbot
        robot;
        
        start_position;
        end_position;
        robot_path;
    end

    methods(Access = protected)
        function setupImpl(obj)
             obj.robot = Robot.soccerbot_base();
             obj.robot_path = obj.robot.getPath(Geometry.transform([0.5 0.0 0]));
        end

        function configuration = stepImpl(obj, has_new_goal, new_goal, ~, new_pose)
            
            if (has_new_goal)
                % TODO wait for final step to finish before moving on
                obj.robot.start_position = new_pose.Pose;
                obj.robot.end_position = Geometry.transform(cell2mat(struct2cell(new_goal))');
                obj.robot_path = obj.robot.getPath(obj.robot.end_position);
                obj.t = 0;
                obj.has_goal = 1;
            end
            
            if (obj.has_goal)
                obj.robot.stepPath(obj.t, obj.robot_path);
            end
            
            configuration = obj.robot.configuration;
        end

        function resetImpl(obj)
            obj.t = 0;
            obj.has_goal = 0;
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

        function [sz,dt,cp] = getDiscreteStateSpecificationImpl(obj,name)
            % Return size, data type, and complexity of discrete-state
            % specified in name
            sz = [1 1];
            dt = "double";
            cp = false;
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Inherited");
        end

        function [dt1] = getOutputDataTypeImpl(obj)
            dt1 = 'double';
        end

        function [cp1] = isOutputComplexImpl(obj)
            cp1 = false;
        end
        
    end
end
