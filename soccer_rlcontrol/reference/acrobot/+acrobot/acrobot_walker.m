classdef acrobot_walker < acrobot.acrobot_control & matlab.System

    properties
        display = 0;
    end

    % Pre-computed constants
    properties(Access = private)
        t;
        fig;
    end

    methods
        % Constructor
        function obj = acrobot_walker(varargin)
            obj = obj@acrobot.acrobot_control(false);

            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.tau_q = [0;0];
            obj.tau = [0;0];
            obj.t = 0;
            if (obj.display && coder.target('MATLAB'))
                obj.fig = figure;
                set(obj.fig, 'Position',  [100, 100, 1500, 700]);
            end
            obj.resetRobot();
            fprintf("%f", obj.step_count);
        end

        function tau = stepImpl(obj, state, collision)

            obj.x = state;
            if (collision)
                obj.step_count = obj.step_count +1;
            end
            obj.t = obj.t + 0.01;
            if (obj.display && coder.target('MATLAB'))
                set(0, 'CurrentFigure', obj.fig)
                obj.show(obj.t);
            end
            tau_vec = obj.getTau(state);
            if (mod(obj.step_count,2) == 0)
                tau = tau_vec(2);
            else
                tau = -tau_vec(2);
            end
        end

        function resetImpl(obj)
            obj.resetRobot();
        end

        function s1 = getOutputSizeImpl(~)
            s1 = 1;
        end

        function d1 = getOutputDataTypeImpl(~)
            d1 = 'double';
        end

        function c1 = isOutputComplexImpl(~)
            c1 = false;

        end

        function c1 = isOutputFixedSizeImpl(~)
            c1 = true;

        end
    end
end
