classdef acrobot_state_estimator < matlab.System
    % Public, tunable properties
    properties
        steps_per_rotation = 2797;
        sample_time = 0.01;
        leg_length = 0.335;
        max_velocity_change = 100;
        flip_direction = 1;
    end

    % Pre-computed constants
    properties(Access = private)
        state = [pi/2;0;0;0];
        cycleCount = 0;
        timeOut = false;
        step_count = 0;
    end

    methods
        % Constructor
        function obj = acrobot_state_estimator(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = public)
        function setupImplPublic(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [state, collision] = stepImplPublic(obj, step, pos, acc, motor_step)
            % Motor Angle
            qm = pi - motor_step / (obj.steps_per_rotation/ (2 * pi));

            % gyro1 and acc1 are always on the ground
            if (mod(step, 2) == 1)
                qm = 2*pi - qm;
            end

            if (obj.flip_direction == -1)
                pos(2) = pos(2) * obj.flip_direction;
                qm = qm * obj.flip_direction + 2 * pi;
            end

            if (pos(2) < 0)
                pos(2) = -pos(2) - pi;
            else
                pos(2) = -pos(2) + pi;
            end

            % Detect jumps in position
            if abs(pos(2) + pi/2 - obj.state(1)) > pi/4
                q1 = obj.state(1);
            else
                q1 = (pos(2) + pi/2);
            end

            q1_dot = (q1 - obj.state(1))/obj.sample_time;
            q2 = (qm - pi);
            q2_dot = (q2 - obj.state(2))/obj.sample_time;

            if(abs(q1_dot) > obj.max_velocity_change)
                q1_dot = obj.state(3);
            end
            if(abs(q2_dot) > obj.max_velocity_change)
                q2_dot = obj.state(4);
            end

            collision = 0;

            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist_to_floor = rc2(2);
            if dist_to_floor < 0
                obj.timeOut = true;
                collision = 1;
                obj.step_count = obj.step_count + 1;
            end


            state = [q1; q2; q1_dot; q2_dot];
            obj.state = state;
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            obj.setupImplPublic();
        end

        function [state, collision] = stepImpl(obj,pos1, pos2, motor_step)
             if mod(obj.step_count, 2) == 0
                pos = pos1;
             else
                pos = pos2;
             end

             [state, collision] = obj.stepImplPublic(obj.step_count, pos, 0, motor_step);
        end

        function [s1, s2] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2 = [1,1];
        end

        function [d1, d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end

        function [c1, c2] = isOutputComplexImpl(~)
            c1 = false;
            c2 = false;
        end

        function [c1, c2] = isOutputFixedSizeImpl(~)
            c1 = true;
            c2 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj,'Type','Discrete',...
              'SampleTime',obj.sample_time,'OffsetTime',0.0);
        end
    end
end
