classdef footpath < Geometry.crotchpath
    properties
        foot_separation = 0.05;     % seperation between feet
        step_lag = 0.2;             % step window ratio complement 
        step_height = 0.015;        % height of step
    end
    
    methods
        function obj = footpath(start_transform, end_transform)
            obj@Geometry.crotchpath(start_transform, end_transform);
            
        end
        
        function position = rightFootPosition(obj, t)
            if (abs(t - obj.duration) < 1e-5)
                position = obj.end_transform.H;
                return;
            end
            step_time = obj.stepTime;
            step_num = fix(t / step_time);
            step_ratio = rem(t, step_time)/step_time;
            previous_step = obj.getStep(step_num);
            next_step = obj.getStep(step_num + 2); % take next step on same side
            
            % check if right
            if rem(step_num + 1, 2) == obj.first_step_left
                position = obj.getStep(step_num+1).H;
                position(2,4) = position(2,4) + obj.foot_separation/2;
                return
            end
            
            % apply step lag
            if step_ratio < obj.step_lag
                position = obj.getStep(step_num).H;
                position(2,4) = position(2,4) + obj.foot_separation/2;
                return
                
            else
                step_ratio = rem(t, step_time*(1-obj.step_lag))/step_time;
            end
            
            % Simple Parabolic trajectory
            % (http://mathworld.wolfram.com/ParabolicSegment.html)
            distance_between_step = Geometry.transform.distance(previous_step, next_step);
            height_per_step = obj.step_height;
            
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
            t1(3,4) = 0.0147; % can be calculated from obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'torso')
            t1(2,4) = t1(2,4) + obj.foot_separation/2;
            
            x = -a + dist;
            y = h * (1 - x^2/a^2);
            
            if (mod(step_num, 2) == obj.first_step_left)
                sidediff = -obj.crotch_sidediff_step;
            else
                sidediff = obj.crotch_sidediff_step;
            end
            
            zdelta = cos(atan2(sidediff, obj.crotch_zdiff_per_step)) * y;
            
            t2 = Geometry.transform([0 0 zdelta], axang2quat([1 0 0 0]));
            
            position = t1 * t2.H;
        end
        
        
        
        function position = leftFootPosition(obj, t)
            if (abs(t - obj.duration) < 1e-5)
                position = obj.end_transform.H;
                return;
            end
            step_time = obj.stepTime;
            step_num = fix(t / step_time);
            step_ratio = rem(t, step_time)/step_time;
            previous_step = obj.getStep(step_num);
            next_step = obj.getStep(step_num + 2); % take next step on same side
            
            
            % check if left
            if rem(step_num + 1, 2) ~= obj.first_step_left
                position = obj.getStep(step_num+1).H;
                position(2,4) = position(2,4) - obj.foot_separation/2;
          
                return
            end
            
            % apply step lag
            if step_ratio < obj.step_lag
                position = obj.getStep(step_num).H;
                position(2,4) = position(2,4) - obj.foot_separation/2;
                return
                
            else
                step_ratio = rem(t, step_time*(1-obj.step_lag))/step_time;
            end
            
            % Simple Parabolic trajectory
            % (http://mathworld.wolfram.com/ParabolicSegment.html)
            distance_between_step = Geometry.transform.distance(previous_step, next_step);
            height_per_step = obj.step_height;
            
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
            t1(3,4) = 0.0147; % can be calculated from obj.robot.getTransform(obj.robot.homeConfiguration, 'left_hip_front', 'torso')
            t1(2,4) = t1(2,4) - obj.foot_separation/2;
            
            x = -a + dist;
            y = h * (1 - x^2/a^2);
            
            if (mod(step_num, 2) == obj.first_step_left)
                sidediff = -obj.crotch_sidediff_step;
            else
                sidediff = obj.crotch_sidediff_step;
            end
            
            zdelta = cos(atan2(sidediff, obj.crotch_zdiff_per_step)) * y;
            
            t2 = Geometry.transform([0 0 zdelta], axang2quat([1 0 0 0]));
            
            position = t1 * t2.H;
        end
        
        function show(obj)
            show@Geometry.crotchpath(obj);
            
            % Draw the foot position
            i = 1;
            for t = 0:0.1:obj.duration
                tfInterp_l(:,:,i) = obj.leftFootPosition(t);
                tfInterp_r(:,:,i) = obj.rightFootPosition(t);
                i = i + 1;
            end
            hold on;
            plotTransforms(tform2trvec(tfInterp_l),tform2quat(tfInterp_l), 'FrameSize', 0.01)
            plotTransforms(tform2trvec(tfInterp_r),tform2quat(tfInterp_r), 'FrameSize', 0.01)
            hold off;
        end
    end
end

