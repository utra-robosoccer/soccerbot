classdef acrobot_torque_control < matlab.System

    % Public, tunable properties
    properties
        steps_per_rotation = 2797;
        p1;
        p2;
        moi_motor;
    end

    properties(DiscreteState)

    end

    % Pre-computed constants
    properties(Access = private)

    end

    methods(Access = public)
        function obj = acrobot_torque_control()
            torque_info = coder.load('data/torque_calculation.mat');
            obj.p1 = torque_info.p1;
            obj.p2 = torque_info.p2;
            obj.moi_motor = torque_info.moi_motor;
        end

        function pwm = getPWM(obj, tau)
            acc = tau / obj.moi_motor;
            if tau > 0
                pwm = (acc + obj.p1(2)) / obj.p1(1);
            else
                pwm = (acc + obj.p2(2)) / obj.p2(1);
            end
            pwm = min(1, max(-1,pwm));
        end
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function pwm = stepImpl(obj, tau)
            pwm = obj.getPWM(tau);
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
end
