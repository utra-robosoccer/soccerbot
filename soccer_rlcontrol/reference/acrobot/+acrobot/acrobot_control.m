classdef acrobot_control < acrobot.acrobot

    properties(Access = protected)
        pheel = [0; 0];      % Position of the heel

        q_field_plotted = 0;
        tau = [0; 0];
        tau_q = [0; 0];
        tau_g = [0; 0];
        holo_point = [0; 0];
        holo_point_dt = [0; 0];

        e_acc = 0;
    end
    properties
        x = zeros(4,1);     % Current x state space

        actual_robot = 0;

        % Controller parameters
        gamma = 0.05; % Used for simulation

        % Real life
        kp = 200;
        ki = 0;
        kd = 5;
        kb = 0.01;
        integral_saturation = 2;
    end

    methods
        function obj = acrobot_control(recalculate)
            obj = obj@acrobot.acrobot(recalculate);
        end

        function resetRobot(obj)
            obj.x = [pi/2; 0; -0.005; -0.05];
            obj.tau = [0; 0];
            obj.holo_point = [0; 0];
            obj.holo_point_dt = [0; 0];
            obj.q_field_plotted = 0;
            obj.step_count = 0;
        end

        function Kp = Kp(obj)
            if (obj.actual_robot)
                Kp = obj.kp;
            else
                Kp = (1/obj.gamma)^2;
            end
        end

        function Kd = Kd(obj)
            if (obj.actual_robot)
                Kd = obj.kd;
            else
                Kd = 2/obj.gamma;
            end
        end

        function Ki = Ki(obj)
            if (obj.actual_robot)
                Ki = obj.ki;
            else
                Ki = 1;
            end
        end

        function [value, isterminal, direction] = dist_to_floor(obj, ~, x, x2_min)
            q1 = x(1);
            q2 = x(2);
            rH = obj.leg_length * [cos(q1); sin(q1)];
            rc2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];
            dist = rc2(2);

            if nargin == 4
                value = [(q1 + q2 - pi/2 - x2_min) dist (pi - x(1)) x(1) pi - abs(x(2))];
                direction = [-1 -1 -1 -1 -1];
                isterminal = [1 1 1 1 1];
            else
                value = [dist (pi - x(1)) x(1) pi - abs(x(2))];
                direction = [-1 -1 -1 -1];
                isterminal = [1 1 1 1];
            end
        end

        function dxdt = physics_step(obj, ~, x, tau)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];

            % Robotics Equation Parameters
            D = acrobot.gen.calc_D(obj.linertia(1), obj.linertia(1), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = acrobot.gen.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = acrobot.gen.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            B = acrobot.gen.calc_B(qdot(2));

            dist = obj.dist_to_floor(0, x);
            if (dist(1) < obj.floor_limit && q(2) < 0)
                obj.tau_q = [0; 0];
            else
                obj.tau_q = D \ tau;
            end
            obj.tau_g = D \ (-C * qdot - P - B);
            qddot_new = obj.tau_g + obj.tau_q;
            dxdt = [qdot; qddot_new];
        end

        function tau = getTau(obj, x)
            q = [x(1); x(2)];
            qdot = [x(3); x(4)];
            dist = obj.dist_to_floor(0, x);
            if (dist(1) < obj.floor_limit && q(2) < 0)
                tau = [0; 0];
                return;
            end

            % Robotics Equation Parameters
            D = acrobot.gen.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2),q(2));
            C = acrobot.gen.calc_C(obj.leg_length, obj.lcom(2), obj.lmass(2), q(2), qdot(1), qdot(2));
            P = acrobot.gen.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q(1), q(2));
            B = acrobot.gen.calc_B(qdot(2));

            % Could use lcurve, codegen workaround
            curve = obj.lcurve;

            [breaks, coefs] = unmkpp(curve.phi);
            phi_curve = mkpp(breaks, coefs);
            [breaks, coefs] = unmkpp(curve.phi_dot);
            phi_dot_curve = mkpp(breaks, coefs);
            [breaks, coefs] = unmkpp(curve.phi_ddot);
            phi_ddot_curve = mkpp(breaks, coefs);

            phi_q2 = ppval(phi_curve, q(2));
            phi_dot_q2 = ppval(phi_dot_curve, q(2));
            phi_ddot_q2 = ppval(phi_ddot_curve, q(2));

            % Code generation hack
            obj.holo_point = [phi_q2(1); q(2)];

            % PD Control
            e = q(1) - phi_q2(1);
            e_dot = qdot(1) - phi_dot_q2(1) * qdot(2);

            if coder.target('MATLAB')
                obj.e_acc = max(-obj.integral_saturation, min(obj.integral_saturation, obj.e_acc + e));
            end

            part1 = [1 -phi_dot_q2(1)] * inv(D) * obj.B;
            part2 = -obj.Ki * obj.e_acc + -obj.Kp * e - obj.Kd * e_dot + phi_ddot_q2(1) * qdot(2)^2 + [1 -phi_dot_q2(1)] * inv(D) * (C * qdot + P + B);

            tau_value = max(-obj.tau_limit, min(obj.tau_limit, part1 \ part2));
            tau = [0; tau_value];

            if coder.target('MATLAB')
                obj.tau = tau;
                obj.tau_q = D \ tau;
            end
%             fprintf("E: %.3f\t Edot: %.3f\t Eacc: %.3f Part 2:%.3f Tau: %.3f\n", e, e_dot, obj.e_acc, part2, obj.tau(2));
        end

        function impact_foot(obj, x)
            q1 = x(1);
            q2 = x(2);
            q1_dot = x(3);
            q2_dot = x(4);

            q = [q1; q2];
            q_dot = [q1_dot; q2_dot];

            De = acrobot.gen.calc_De(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), q1, q2);
            E = acrobot.gen.calc_E(obj.leg_length, obj.leg_length, q1, q2);
            dUde = acrobot.gen.calc_dUde(obj.leg_length, q1);
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling

            qp = wrapTo2Pi(T * q + [-pi; 0]);
            qp_dot = [T zeros(2,2)] * (delta_qedot * q_dot) * obj.lcurve.energy_loss;

            % Collision with floor?
            rend_dot = acrobot.gen.calc_J(obj.leg_length, obj.leg_length, qp(1), qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                obj.x = [qp; -qp_dot];
            else
                obj.x = [qp; qp_dot];
            end

            disp(strcat("Impact Velocity Pre: Q: ", num2str(norm(q_dot)), " D: ", num2str(norm(obj.lcurve.xm(3:4)))));
            disp(strcat("Impact Velocity Post: Q: ", num2str(norm(qp_dot)), " D: ", num2str(norm(obj.l2curve.xp(3:4)))));
            disp(strcat("Impact Angle Error Pre: ", num2str(angdiff(atan2(obj.lcurve.xm(4), obj.lcurve.xm(3)), atan2(q_dot(2), q_dot(1))))));
            disp(strcat("Impact Angle Error Post: ", num2str(angdiff(atan2(obj.lcurve.xp(4), obj.lcurve.xp(3)), atan2(qp_dot(2), qp_dot(1))))));

            % Increase Step Count
            obj.step_count = obj.step_count + 1;

            % Change heel location
            rH = obj.leg_length * [cos(q1); sin(q1)];                       % Hip position
            step_diff = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];     % Swing foot position
            obj.pheel(1) = obj.pheel(1) + step_diff(1);

        end

        function plotRobot(obj)
            subplot(2,3,1);
            cla;
            q1 = obj.x(1);
            q2 = obj.x(2);

            rc1 = (obj.leg_length - obj.lcom(1)) * [cos(q1); sin(q1)] + obj.pheel;
            rH = obj.leg_length * [cos(q1); sin(q1)] + obj.pheel;
            rc2 = rH + obj.lcom(2) * [cos(q1+q2); sin(q1+q2)];
            pH2 = rH + obj.leg_length * [cos(q1+q2); sin(q1+q2)];

            hold off;
            plot(rc1(1), rc1(2), '.', 'markersize',20,'color','b');     % Stance leg mass
            hold on;
            plot(rc2(1), rc2(2), '.', 'markersize',20,'color','b');     % Swing leg mass
            r1 = line([obj.pheel(1);rH(1)],[obj.pheel(2),rH(2)]);       % heel1 to hip
            r2 = line([rH(1),pH2(1)],[rH(2),pH2(2)] );                  % hip to heel2

            r1.Color = 'blue';
            r2.Color = 'black';

            grid on;
            axis equal;
            ylim([-0.5, 1]);
            xlim([-0.5, 2]);
            title("Robot Position")
        end

        function plotFields(obj)
            % Plotting the subplot field
            if ~obj.q_field_plotted
                step_count = obj.step_count;

                subplot(2,3,[2,5]);
                obj.step_count = 0;
                plotHolonomicCurve(obj, obj.c1);
                hold on;
                plotHolonomicCurve(obj, obj.pre_c, 'y');
                hold on;

                subplot(2,3,[3,6]);
                obj.step_count = 1;
                plotHolonomicCurve(obj, obj.c2);
                hold on;

                obj.q_field_plotted = 1;
                obj.step_count = step_count;
            end

            if (rem(obj.step_count,2) == 0)
                subplot(2,3,[2,5]);
            else
                subplot(2,3,[3,6]);
            end

            q1 = obj.x(1);
            q2 = obj.x(2);

            plot(q1(1), q2(1), '.', 'markersize',5,'color',[0 0 0]);
%             plot(obj.holo_point(1), obj.holo_point(2), '.', 'markersize',5,'color',[0 1 0]);
            quiver(q1(1), q2(1), obj.tau_q(1) * 0.005, obj.tau_q(2) * 0.005);
        end

        function plotTau(obj, t)
            subplot(2,3,4);
            hold on;
            plot(t, obj.tau(2), '+', 'markersize',5,'color','m');
            ylabel('Tau N*m');
            xlabel('Time (s)');
            title("Torque over Time Plot")
            ylim([-obj.tau_limit - 0.1 obj.tau_limit + 0.1]);
            grid on;
        end

        function show(obj, t)
            % Plot Robot
            obj.plotRobot();

            % Plot q1,q2 field
            obj.plotFields();

            % Plotting tau
            obj.plotTau(t);

            drawnow;
        end
    end
end
