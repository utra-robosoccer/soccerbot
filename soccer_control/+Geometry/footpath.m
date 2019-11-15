classdef footpath < Geometry.path
    properties
        foot_separation = 0.05;  
        crotch_zdiff_per_step = 0.02;
        crotch_sidediff_step = 0.02;
        crotch_rotation_per_step = pi/30;
        
        first_step_left = 0;
    end
    
    methods
        function obj = footpath(start_transform, end_transform)
            obj = obj@Geometry.path(start_transform, end_transform);
            
            % Calculate the foot for the first step (based on destination)
            axang = quat2axang(obj.start_transform.orientation);
            theta1 = axang(4);
            diff_transform = end_transform.H / start_transform.H;
            theta2 = atan2(diff_transform(2,4), diff_transform(1,4));
            if (mod(theta2 - theta1, 2*pi) > pi)
                obj.first_step_left = 0;
            else
                obj.first_step_left = 1;
            end
            
        end
        
        
        function position = crotchPosition(obj, t)
            if (abs(t - obj.duration) < 1e-5)
                position = obj.end_transform.H;
                return;
            end
            step_time = obj.stepTime;
            step_num = fix(t / step_time);
            step_ratio = rem(t, step_time)/step_time;
            previous_step = obj.getStep(step_num);
            next_step = obj.getStep(step_num + 1);
            
            % Simple Parabolic trajectory
            % (http://mathworld.wolfram.com/ParabolicSegment.html)
            distance_between_step = Geometry.transform.distance(previous_step, next_step);
            height_per_step = norm(obj.crotch_zdiff_per_step, obj.crotch_sidediff_step);
            
            h = height_per_step;
            a = distance_between_step / 2;
            
            length_arc = sqrt(a^2 + 4*h^2) + a^2/2/h * asinh(2*h/a);
            length_so_far = step_ratio * length_arc;
            half_length = length_arc / 2;
            
            diff_length = abs(length_so_far - half_length);
            syms x;
            
            lindist = sqrt(a^2 + h^2);
            guess = acosh(sqrt(((abs(step_time*lindist - lindist/2))*2*h/a^2)^2 + 1));
%             tmp = guess;
            tmp = double(vpasolve(diff_length*4*h/a^2 == x + sinh(x)*cosh(x), x, guess));
            tmp = sqrt(cosh(tmp)^2 - 1)*a^2/2/h;
            if (length_so_far < half_length)
                dist = distance_between_step/2 - tmp;
            else
                dist = distance_between_step/2 + tmp;
            end
            
            position_time = dist / distance_between_step * step_time;
            if position_time < 0
                position_time = 0;
            end
            [t1, ~, ~] = transformtraj(previous_step.H,next_step.H,[0 step_time],position_time);
            
            x = -a + dist;
            y = h * (1 - x^2/a^2);
            
            if (mod(step_num, 2) == obj.first_step_left)
                sidediff = -obj.crotch_sidediff_step;
            else
                sidediff = obj.crotch_sidediff_step;
            end
            
            
            zdelta = cos(atan2(sidediff, obj.crotch_zdiff_per_step)) * y;
            ydelta = sin(atan2(sidediff, obj.crotch_zdiff_per_step)) * y;
            
            if (mod(step_num, 2) == obj.first_step_left)
                thetadelta = y / height_per_step * obj.crotch_rotation_per_step;
            else
                thetadelta = - y / height_per_step * obj.crotch_rotation_per_step;
            end
            
            t2 = Geometry.transform([0 ydelta zdelta], axang2quat([1 0 0 thetadelta]));
            
            position = t1 * t2.H;
        end
        
        function show(obj)
            show@Geometry.path(obj);
            
            % Draw the crotch position
            i = 1;
            for t = 0:0.1:obj.duration
                tfInterp(:,:,i) = obj.crotchPosition(t);
                i = i + 1;
            end
            plotTransforms(tform2trvec(tfInterp),tform2quat(tfInterp), 'FrameSize', 0.01)
        end
    end
end

