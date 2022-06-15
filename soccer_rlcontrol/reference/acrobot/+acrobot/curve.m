classdef curve < handle
    %CURVE Summary of this class goes here
    %   Parameters for a single curve

    properties

        % Curve Parameters (Can vary with alternating foot)
        beta = pi/5.8;              % Angle which to hit the ground
        impact_angle = 0.0;         % Angle to impact the ground
        impact_velocity = 0.0;      % In terms of q1 q2 norm
        energy_loss = 1.0;

        % Computed Curve parameters
        xm; % Robot state pre-impact [q1, q2, q1dot, q2dot] (Starting point)
        xp; % Robot state post-impact [q1, q2, q1dot, q2dot] (Ending point)

        % [t1 tau1 t2 tau2] % Timeseries of taus
        tau_m_guess;
        tau_m;

        phi;         % Function phi(q2) = q1
        phi_dot;
        phi_ddot;

    end

    methods
        function obj = curve(beta, impact_angle, impact_velocity, energy_loss, tau_m_guess)
            obj.beta = beta;
            obj.impact_angle = impact_angle;
            obj.impact_velocity = impact_velocity;
            obj.energy_loss = energy_loss;
            obj.tau_m_guess = tau_m_guess;

            obj.phi = struct([]);
            obj.phi_dot = struct([]);
            obj.phi_ddot = struct([]);
        end
    end
end
