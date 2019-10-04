classdef LegTransform < Transform
    %Transform a general position on the field
    
    properties
    end
    
    methods
        function obj = LegTransform(position, orientation)
            %TODO look how to inherit in maltab constructor
            obj.position = position;
            if nargin == 2
                obj.orientation = orientation;
            end
        end
        
        function angles = InverseKinematics(obj)
            
            % lengths of links
            l1 = dh(3, 3); l2 = dh(4, 3); l3 = dh(6, 3);

            % polar distance and angle to end effector
            r = sqrt(x6^2 + y6^2);
            t = atan2(y6, x6);

            q = zeros(6, 1);

            % Hip angle to ensure angle of foot
            q(1) = -g;

            % Position of right before the end (ankle)
            z5 = z6 + l3;

            % Shift foot so that it is inline with desired position
            d = r * sin(g - t);
            q(2) = atan2(d, -z5);
            q(6) = -q(2);

            % Distance to ankle
            a = sqrt(d^2 + z5^2); % "Vertical"
            b = r * cos(g - t); % "Horizontal"
            dab = sqrt(a^2 + b^2);

            % is this position possible
            if (dab < l1 - l2 || dab > l1 + l2)
                error('Goal position is unreachable');
            end

            % use cosine  law to solve angles of triangle
            C1 = (l1^2 + dab^2 - l2^2)/(2*l1*dab);
            t1 = atan2(sqrt(1 - C1^2), C1);
            C2 = (l1^2 + l2^2 - dab^2)/(2*l1*l2);
            t2 = atan2(sqrt(1 - C2^2), C2);
            t3 = pi - t1 - t2;

            % construct the two possible q from data available
            qab = -atan2(a, b) + pi/2;
            q_a = [ qab + t1; t2 - pi; -qab + t3];
            q_b = [ qab - t1; pi - t2; -qab - t3];

            % choose the option closest to the previous answer
            if norm(q_a - q0(3:5)') < norm(q_b - q0(3:5)')
                q(3:5) = q_a;
            else
                q(3:5) = q_b;
            end
        end
    end
end

