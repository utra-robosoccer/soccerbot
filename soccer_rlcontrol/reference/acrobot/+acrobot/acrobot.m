classdef acrobot < handle

    properties(Access = private)
        % Computed Curve Parameters
        p1;
        p2;
        p3;
        p4;
    end
    properties(Access = protected)
        robot;

        % Convenience values
        mass = zeros(2,1);
        com = zeros(2,1);
        inertia = zeros(2,1);

        % VHC Parameters
        B = [0; 1];

        pre_c = acrobot.curve(pi/9.5, pi*0.25, 1.5, 0.5, [0.6052, -0.0754, 0.8631, 0.0658]); % First Step
        c1 = acrobot.curve(pi/9.5, pi*0.25, 1.5, 0.5, [0.0232, -0.2047, 0.5594, 0.1340]); % Third Step
        c2 = acrobot.curve(pi/9.5, pi*0.25, 1.5, 0.5, [0.0602, -0.2710, 0.4843, 0.2053]); % Second Step

        % Physical Parameters
        g = 9.81;

        % Mechanical Parameters
        leg_length = 0;
        foot_radius = 0.018;
        motor_friction = 0.01;

        % Energy Loss
        fall_duration = 1.8;            % Max Fall duration (not desired)
        tau_limit = 0.33;
        floor_limit = 0.0;             % Distance to the floor where no torque is allowed

    end
    properties(Access = public)
        angle_limit = pi/20;
        step_count = 0;
    end

    methods
        function obj = acrobot(recalculate)
            if (recalculate == true)
                obj.calcCurves();
                obj.updateCurves();
            else
                obj.updateCurves();
            end
        end

        function calcCurves(obj)
            obj.robot = importrobot("acrobot_description/models/acrobot.urdf");
            obj.robot.showdetails
            t = obj.robot.getTransform(obj.robot.homeConfiguration, 'base_link', 'leg1');
            obj.leg_length = t(1,4) + obj.foot_radius;

            % Calculations
            for i = 1:2
                obj.mass(i) = obj.robot.Bodies{i}.Mass;
                obj.com(i) = norm(obj.robot.Bodies{i}.CenterOfMass(1));
                obj.inertia(i) = obj.robot.Bodies{i}.Inertia(2);
            end

            % Create Robot Equation handles
            obj.solveRoboticsEquation();

            % Solve for the curve for both legs
            obj.calcRobotStates();

            obj.step_count = 0;
            obj.calcHolonomicCurves(obj.pre_c, true);
            obj.step_count = 0;
            obj.calcHolonomicCurves(obj.c1, true);
            obj.step_count = 1;
            obj.calcHolonomicCurves(obj.c2, true);
            obj.step_count = 0;

            pre_c = struct(obj.pre_c);
            c1 = struct(obj.c1);
            c2 = struct(obj.c2);
            leg_length = obj.leg_length;
            mass = obj.mass;
            com = obj.com;
            inertia = obj.inertia;

            save("data/curve_parameters", "leg_length", "mass", "com", "inertia", "pre_c", "c1", "c2");
        end

        function updateCurves(obj)

            % Curves
            data = coder.load("data/curve_parameters");
            obj.pre_c = data.pre_c;
            obj.c1 = data.c1;
            obj.c2 = data.c2;

            obj.leg_length = data.leg_length;
            obj.mass = data.mass;
            obj.com = data.com;
            obj.inertia = data.inertia;

        end


        function mass = lmass(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            mass = obj.mass(num);
        end

        function com = lcom(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            com = obj.com(num);
        end

        function inertia = linertia(obj, num)
            if rem(obj.step_count,2) == 1
                if num == 2
                    num = 1;
                else
                    num = 2;
                end
            end
            inertia = obj.inertia(num);
        end

        function curve = lcurve(obj) %#codegen
            if obj.step_count == 0
                curve = obj.pre_c;
            elseif rem(obj.step_count,2) == 0
                curve = obj.c1;
            else
                curve = obj.c2;
            end
        end

        function curve = l2curve(obj)
            if rem(obj.step_count,2) == 1
                curve = obj.c1;
            else
                curve = obj.c2;
            end
        end

        function X = clipCurve(obj, X)
            x = X(round(length(X)/2),2);
            for i = round(length(X)/2):-1:1
                if X(i,2) < x
                    break
                end
                x = X(i,2);
            end

            for j = round(length(X)/2):1:length(X)
                dist = obj.dist_to_floor(0, X(j,:));
                if (dist(1) < obj.floor_limit)
                    break;
                end
            end
            X = X(i+1:j,:);
            X = X(1:2:end,:); % Reduce size
        end

        function calcRobotStates(obj)
            % Post impact for one foot is pre-impact for next foot
            temp = obj.step_count;

            % First Step
            obj.step_count = 0;
            obj.pre_c.xp = [pi/2; 0; 0; -0.05];
            pre_c_qm = [(pi - obj.pre_c.beta)/2; obj.pre_c.beta - pi]; % Joint angles pre impact
            pre_c_qp = [(pi + obj.pre_c.beta)/2; pi - obj.pre_c.beta]; % Joint angles pre impact
            [~, pre_c_w] = obj.getImpactVelocities(pre_c_qm, pre_c_qp, obj.pre_c.impact_angle, obj.pre_c.impact_velocity);
            obj.pre_c.xm = [pre_c_qm; pre_c_w];

            % Second Step

            % Heavy foot on the ground
            c1_qp = [(pi + obj.c2.beta)/2; pi - obj.c2.beta]; % Joint angles post impact
            c1_qm = [(pi - obj.c1.beta)/2; obj.c1.beta - pi]; % Joint angles pre impact

            % Light foot on the ground
            c2_qp = [(pi + obj.c1.beta)/2; pi - obj.c1.beta]; % Joint angles post impact
            c2_qm = [(pi - obj.c2.beta)/2; obj.c2.beta - pi]; % Joint angles pre impact


            [c2_v, c1_w] = obj.getImpactVelocities(c1_qm, c2_qp, obj.c1.impact_angle, obj.c1.impact_velocity);
            obj.step_count = 1;
            [c1_v, c2_w] = obj.getImpactVelocities(c2_qm, c1_qp, obj.c2.impact_angle, obj.c2.impact_velocity);

            % Then ground to ground
            obj.c1.xp = [c2_qp; c2_v];
            obj.c1.xm = [c2_qm; c2_w];

            % Then again ground to ground
            obj.c2.xm = [c1_qm; c1_w];
            obj.c2.xp = [c1_qp; c1_v];

            obj.step_count = temp;
        end

        % objective [dist to final point; velocity to final point]
        function objective = calcHolonomicCurveHelper(obj, xm, xp, tau_m, q2_min)
            if (nargin == 5)
                [X, fail] = obj.getFallingCurve(xm, obj.fall_duration, tau_m, q2_min);
            else
                [X, fail] = obj.getFallingCurve(xm, obj.fall_duration, tau_m);
            end

            dist = (xp' - X(end,:)).^2;
            objective = [dist(1) + dist(2) dist(3) + dist(4) abs(angdiff(atan2(X(end,4), X(end,3)), atan2(xp(4), xp(3))))];
            if (fail)
                objective = objective * 10;
            end

            if (~fail)
                %plot(X(:,1),X(:,2), 'color', [0,0,0.5,0.06]);
                %hold on;
                disp(objective);
            end
        end

        function calcHolonomicCurves(obj, curve, optimize)

            figure;
            hold on;
            plot(curve.xm(1), curve.xm(2),'o', 'MarkerSize',5,'color','k');
            plot(curve.xp(1), curve.xp(2),'o', 'MarkerSize',5,'color','k');
            q1_range = 0:0.05:pi;
            plot(q1_range, -2 * q1_range + 2 * pi,'color','cyan');
            plot(q1_range, -2 * q1_range, 'color','cyan');

            % Plot 90 degree fall curve
            rend = acrobot.gen.calc_rend(obj.leg_length, obj.leg_length, curve.xm(1), curve.xm(2));
            rend_range = [sin(curve.impact_angle); cos(curve.impact_angle)] * (0:0.01:0.05);
            qup = zeros(2, length(rend_range));
            for i = 1:length(rend_range)
                qup(:,i) = acrobot.gen.calc_qd(obj.leg_length, obj.leg_length, rend(1) + rend_range(2, i), rend(2) + rend_range(2, i));
            end
            plot(qup(1,:), qup(2,:));

            axis equal;
            grid minor;
            xlim([0, pi]);
            ylim([-pi, pi]);
            xlabel('q1');
            ylabel('q2');

            % Global Search
            weight = [1,0.1,0.005];
            x0 = curve.tau_m_guess;

            % Local Search
            fun = @(tau_m) obj.calcHolonomicCurveHelper(curve.xp, curve.xm, tau_m);
            goal = [0.001,0.05,0.1];
            lb = [0,-obj.tau_limit,0,-obj.tau_limit];
            ub = [obj.fall_duration,obj.tau_limit,obj.fall_duration,obj.tau_limit];
            A = [1 0 -1 0]; % time 1 < time 2
            b = 0;
            Aeq = [];
            beq = [];
            nonlcon = [];

            if (optimize)
                [x,fval] = gamultiobj(fun,4,A,b,Aeq,beq,lb,ub,nonlcon, ...
                    optimoptions('gamultiobj','PopulationSize', 100, 'UseParallel', true));
                fval = fval .* weight;
                [~, index] = min(vecnorm(fval,2,2));
                x0 = x(index,:);
                options = optimoptions('fgoalattain','DiffMaxChange', 0.001, 'MaxFunctionEvaluations', 5e3, 'UseParallel', true);
                curve.tau_m = fgoalattain(fun,x0,goal,weight,A,b,Aeq,beq,lb,ub,nonlcon,options);
            else
                curve.tau_m = x0;
            end

            X = obj.getFallingCurve(curve.xp, obj.fall_duration, curve.tau_m);
            X = obj.clipCurve(X);
            [~,idx] = unique(X(:,2));
            X = X(idx,:);
            curve.phi = spline(X(:,2), X(:,1));
            curve.phi_dot = fnder(curve.phi,1);
            curve.phi_ddot = fnder(curve.phi,2);
            plot(X(:,1), X(:,2), 'Color', 'red')
            hold off;
        end

        function [v, w] = getImpactVelocities(obj, qm, qp, impact_angle, impact_velocity)

            rend = acrobot.gen.calc_rend(obj.leg_length, obj.leg_length, qm(1), qm(2));
            rend = rend + [cos(impact_angle) * 0.01; sin(impact_angle) * 0.01];
            qm_pre = acrobot.gen.calc_qd(obj.leg_length, obj.leg_length, rend(1), rend(2));
            w = (qm - qm_pre)/norm(qm-qm_pre) * impact_velocity;

            % Post impact calculations
            De = acrobot.gen.calc_De(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), obj.lmass(1), obj.lmass(2), qm(1), qm(2));
            E = acrobot.gen.calc_E(obj.leg_length, obj.leg_length, qm(1), qm(2));
            dUde = acrobot.gen.calc_dUde(obj.leg_length, qm(1));
            last_term = [eye(2); dUde];

            delta_F = -(E/De*E')\E*last_term;
            delta_qedot = De\E'*delta_F + last_term;
            T = [1 1; 0 -1]; % Relabelling
            qp_dot = [T zeros(2,2)] * (delta_qedot * w) * obj.lcurve.energy_loss;
            rend_dot = acrobot.gen.calc_J(obj.leg_length, obj.leg_length, qp(1), qp(2)) * qp_dot;
            if (rend_dot(2) < 0)
                v = -qp_dot;
            else
                v = qp_dot;
            end
        end

        function fnplti(~, fnct)
            X = -pi:0.01:pi;
            Y = fnval(fnct, X);
            plot(Y, X, 'LineWidth',2);
        end

        function plotHolonomicCurve(obj, curve, color)
            if nargin == 2
                color = 'k';
            end

            obj.plotQField();
            hold on;
            obj.plotPField();
%            obj.plotControllerSensitivityField();

            plot(curve.xp(1), curve.xp(2),'o', 'MarkerSize',5,'color',color);
            plot(curve.xm(1), curve.xm(2),'o', 'MarkerSize',5,'color',color);

            quiver(curve.xp(1), curve.xp(2), curve.xp(3), curve.xp(4), 'LineWidth', 2, 'MaxHeadSize', 0.4);
            quiver(curve.xm(1), curve.xm(2), curve.xm(3), curve.xm(4), 'LineWidth', 2, 'MaxHeadSize', 0.4);

            obj.fnplti(curve.phi);

            xlabel('q1')
            ylabel('q2')
            axis equal
            xlim([0, pi])
            hold off;
        end

        function plotQField(obj)
            q1_range = 0:0.05:pi;
            q2_range = -pi:0.05:pi;
            [BX,BY] = meshgrid(q1_range,q2_range);

            BU = zeros(size(BX));
            BV = zeros(size(BY));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(BX,1)
                for j=1:size(BY,2)
                    temp = acrobot.gen.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, BY(i,j)) \ obj.B;
                    BU(i,j) = temp(1);
                    BV(i,j) = temp(2);
                end
            end

%             quiver(obj.BX,obj.BY,obj.BU,obj.BV) %orbits
            streamslice(BX,BY,BU,BV,'color','cyan'); %orbits

            hold on;

            % Plot impact surfaces S+, S-
            plot(q1_range, -2 * q1_range + 2 * pi,'color','magenta', 'LineWidth', 2);
            plot(q1_range, -2 * q1_range, 'color','magenta', 'LineWidth', 2);
            xlim([0, pi]);
            ylim([-pi, pi]);

            hold off;
        end

        function plotPField(obj)
            % Plot vector field inv(D)*B
            finity = 0.2;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);

            R = zeros(size(X1));
            Z = zeros(size(X2));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = acrobot.gen.calc_D(obj.linertia(1), obj.linertia(1), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X2(i,j));
                    P = acrobot.gen.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X1(i,j), X2(i,j));
                    temp = D \ -P;
                    R(i,j) = temp(1);
                    Z(i,j) = temp(2);
                end
            end
            quiver(X1,X2,R,Z,'color',[1 0.6 1])
        end

        % tau_m = [t1, tau_1, t2, tau_2]
        function [X, fail] = getFallingCurve(obj, xs, tend, tau, q2_min)
            fail = 0;
            t = 0;

            xs = xs';
            X = zeros(0,4);

            tau = [0 tau tend];

            if nargin == 5
                options = odeset('Events',@(t,x)obj.dist_to_floor(t, x, q2_min), 'RelTol', 1e-2, 'AbsTol', 1e-5);
            else
                options = odeset('Events',@(t,x)obj.dist_to_floor(t, x), 'RelTol', 1e-2, 'AbsTol', 1e-5);
            end

            for i = 1:length(tau)/2
                tau_curr = tau(i*2-1);
                t_max = tau(i*2);

                if t == t_max
                    continue;
                end

                % Run Tau until collision or angle reached
                [tt, XX, ~, xe, ie] = ode45(@(t, x) obj.physics_step(t, x, [0; tau_curr]), [t t_max], xs, options);
                X = [X; XX];

                % Collided with something or never hit the ground
                if (i < length(tau)/2 && ~isempty(ie) || ...
                    i == length(tau)/2 && isempty(ie) || ...
                    i == length(tau)/2 && length(ie) ~= 1 || ...
                    i == length(tau)/2 && ie(1) ~= 1)
                    fail = 1;
                    return
                end
                t = tt(end);
                xs = XX(end,:);
            end
            X = [X; xe];
        end

        function plotControllerSensitivityField(obj)
            % Plot vector field inv(D)*B
            finity = 0.1;
            q1_range = 0:finity:pi;
            q2_range = -pi:2*finity:pi;
            [X1,X2] = meshgrid(q1_range,q2_range);

            R = zeros(size(X1));

            % Dinv * B
            m1 = obj.lmass(1);
            m2 = obj.lmass(2);
            for i=1:size(X2,1)
                for j=1:size(X2,2)
                    D = acrobot.gen.calc_D(obj.linertia(1), obj.linertia(2), obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X2(i,j));
                    P = acrobot.gen.calc_P(obj.g, obj.leg_length, obj.lcom(1), obj.lcom(2), ...
                                    m1, m2, X1(i,j), X2(i,j));

                    temp1 = D \ obj.B;
                    temp2 = D \ -P;

                    t2_hat = temp2 / norm(temp2);
                    a1 = dot(temp1, t2_hat) * t2_hat;
                    R(i,j) = norm(temp1 - a1);
                end
            end

            surf(X1, X2, R, 'EdgeColor', 'none');

            alpha 0.15
            hold on;
            plot(q1_range, -2 * q1_range + 2 * pi,'color','black');
            plot(q1_range, -2 * q1_range, 'color','black');
            xlim([0, pi]);
            ylim([-pi, pi]);
            title('Controller Sensitivity')
            xlabel('q1')
            ylabel('q2')
            view(0,90)
        end

        function solveRoboticsEquation(obj)
            % Equations of Motion

            syms q1 q2 q1dot q2dot q1ddot q2ddot real
            syms l1 l2 lc1 lc2 i1 i2 real
            syms m1 m2 g real

            q = [q1; q2];
            qdot = [q1dot; q2dot];
            qddot = [q1ddot; q2ddot];

            % Positions
            rc1 = (l1 - lc1) * [cos(q1); sin(q1)];
            rH = l1 * [cos(q1); sin(q1)];
            rc2 = rH + lc2 * [cos(q1+q2); sin(q1+q2)];
            rend = rH + l2 * [cos(q1+q2); sin(q1+q2)];

            % Velocities
            rc1dot = simplify(jacobian(rc1,q) * qdot);
            rc2dot = simplify(jacobian(rc2,q) * qdot);
            renddot = simplify(jacobian(rend,q));

            w01 = q1dot;
            w02 = (q1dot+q2dot);

            % Kinetic and Potential Energy
            T1 = 0.5 * m1 * (rc1dot' * rc1dot) + 0.5 * i1 * w01^2;
            T2 = 0.5 * m2 * (rc2dot' * rc2dot) + 0.5 * i2 * w02^2;
            T = T1 + T2;
            U1 = m1 * g * rc1(2);
            U2 = m2 * g * rc2(2);
            U = U1 + U2;
            L = simplify(T - U);
            EE = T + U;
            dLdq = jacobian(L,q)';
            dLdqdot = jacobian(L,qdot)';
            ddtdLdqdot = jacobian(dLdqdot,q) * qdot + jacobian(dLdqdot,qdot) * qddot;
            Tau = simplify(ddtdLdqdot - dLdq);

            % Find the C, D, P, B Matrix
            syms C D P B real
            [D, b] = equationsToMatrix(Tau, qddot);
            P = -subs(b, [q1dot, q2dot], [0, 0]);
            C = sym(zeros(length(q)));
            for k = 1:size(q)
                for j = 1:size(q)
                    for i = 1:size(q)
                        Qijk = 0.5*(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)));
                        C(k,j) = C(k,j) + simplify(Qijk) * qdot(i);
                    end
                end
            end
            B = [0; obj.motor_friction] .* qdot;

            % Impact Map
            % Solving for EOM and impact map following
            % "Feedback Control of Dynamic Bipedal Robot Locomotion" by Grizzle, p. 55
            % Based on the 3 link model on p. 67 and the paper "Asymptotically Stable
            % Walking for Biped Robots: Analysis via Systems with Impulse Effects"

            syms q3 q4 q3dot q4dot q3ddot q4ddot real

            qe = [q1; q2; q3; q4];
            qedot = [q1dot; q2dot; q3dot; q4dot];
            qeddot = [q1ddot; q2ddot; q3ddot; q4ddot];

            % Positions
            e = [q3; q4];
            rc1e = rc1 + e;
            rHe = rH + e;
            rc2e = rc2 + e;
            rende = rend + e;

            rc1edot = simplify(jacobian(rc1e, qe) * qedot);
            rc2edot = simplify(jacobian(rc2e, qe) * qedot);

            % Kinetic and Potential Energy
            Te1 = 0.5 * m1 * (rc1edot' * rc1edot) + 0.5 * i1 * w01^2;
            Te2 = 0.5 * m2 * (rc2edot' * rc2edot) + 0.5 * i2 * w02^2;
            Te = Te1 + Te2 ;
            Ue1 = m1 * g * rc1e(2);
            Ue2 = m2 * g * rc2e(2);
            Ue = Ue1 + Ue2;
            Le = simplify(Te - Ue);

            % Finding the EOM
            dLedq = jacobian(Le,qe)';
            dLedqdot = jacobian(Le,qedot)';
            ddtdLedqdot = simplify(jacobian(dLedqdot, qe) * qedot + jacobian(dLedqdot, qedot) * qeddot);
            Taue = simplify(ddtdLedqdot - dLedq);

            % Solve for D matrix
            [De, ~] = equationsToMatrix(Taue, qeddot);

            % Upsilons
            E = simplify(jacobian(rende, qe));
            dUde = simplify(jacobian(rHe,q));

            % Inverse kinematics
            syms x y r q1d q2d qd real;
            q2d = acos((l1^2 + l2^2 - (x^2 + y^2))/l1/l2/2) - pi;
            q1d = atan2(y,x) + (pi - (pi + q2d))/2;
            qd = [q1d; q2d];

            % Add the matlab functions
            matlabFunction(renddot, 'File', '+acrobot/+gen/calc_J');
            matlabFunction(D, 'File', '+acrobot/+gen/calc_D');
            matlabFunction(C, 'File', '+acrobot/+gen/calc_C');
            matlabFunction(P, 'File', '+acrobot/+gen/calc_P');
            matlabFunction(B, 'File', '+acrobot/+gen/calc_B');
            matlabFunction(De, 'File', '+acrobot/+gen/calc_De');
            matlabFunction(E, 'File', '+acrobot/+gen/calc_E');
            matlabFunction(dUde, 'File', '+acrobot/+gen/calc_dUde');
            matlabFunction(EE, 'File', '+acrobot/+gen/calc_EE');
            matlabFunction(qd, 'File', '+acrobot/+gen/calc_qd');
            matlabFunction(rend, 'File', '+acrobot/+gen/calc_rend');
        end
    end
end
