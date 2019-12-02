classdef footpath < Geometry.path
    properties
        half_to_full_step_time_ratio = 0.7; % Duration difference between half and full step
        
        foot_separation = 0.04;     % seperation between feet and center
        step_height = 0.03;        % height of step
        step_outwardness = 0.025;
        step_rotation = -0.15;
        
        foot_center_to_floor;
    end
    
    methods
        function obj = footpath(start_transform, end_transform, foot_center_to_floor)
            obj@Geometry.path(start_transform, end_transform);
            obj.foot_center_to_floor = foot_center_to_floor;
        end
        
        function half_step_time = half_step_time(obj)
            half_step_time = obj.full_step_time * obj.half_to_full_step_time_ratio;
        end
        
        function num_steps = num_steps(obj)
            num_steps = obj.bodyStepCount + 1;
        end
        
        function full_step_time = full_step_time(obj)
            total_step_time = obj.duration;
            full_step_time = total_step_time / (2 * obj.half_to_full_step_time_ratio + (obj.num_steps - 2));
        end
        
        function [step_num, right_foot_step_ratio, left_foot_step_ratio] = footHeightRatio(obj, t, post_pre_settings)
            full_step_time = obj.full_step_time;
            half_step_time = obj.half_step_time;
            
            post_step_time = obj.post_footstep_ratio * full_step_time;
            pre_step_time = obj.pre_footstep_ratio * full_step_time;
            
            % post_pre_settings: 0 = with post and pre
            % post_pre_settings: 1 = only post and pre on last ones
            % post_pre_settings: 2 = only post
            % post_pre_settings: 3 = no post nor pre
            if (nargin == 3)
                if (post_pre_settings == 1)
                    if t < half_step_time
                        pre_step_time = 0;
                    elseif t > obj.duration - half_step_time
                        post_step_time = 0;
                    else
                        post_step_time = 0;
                        pre_step_time = 0;
                    end
                elseif (post_pre_settings == 2)
                    pre_step_time = 0;
                    post_step_time = -post_step_time;
                elseif (post_pre_settings == 3)
                    post_step_time = 0;
                    pre_step_time = 0;
                end
            end
            
            last_foot_same = mod(obj.num_steps, 2);
            
            step_num = -1;

            % First foot
            if t < half_step_time
                if (t < post_step_time)
                    first_foot_step_ratio = 0;
                elseif(t > (half_step_time - pre_step_time))
                    first_foot_step_ratio = 1;
                else
                    first_foot_step_ratio = (t - post_step_time) / (half_step_time - post_step_time - pre_step_time);
                end
            elseif last_foot_same && t > obj.duration - half_step_time
                adjusted_step_time = t - (obj.duration - half_step_time);
                if (adjusted_step_time < post_step_time)
                     first_foot_step_ratio = 0;
                elseif(adjusted_step_time > (half_step_time - pre_step_time))
                    first_foot_step_ratio = 1;
                else
                    first_foot_step_ratio = (adjusted_step_time - post_step_time) / (half_step_time - post_step_time - pre_step_time);
                end
            else
                adjusted_step_time = t - half_step_time;
                step_num = fix(adjusted_step_time/full_step_time);
                adjusted_step_time = adjusted_step_time - step_num * full_step_time;
                if (mod(step_num, 2) == 0)
                    first_foot_step_ratio = 0;
                else
                    if (adjusted_step_time < post_step_time)
                        first_foot_step_ratio = 0;
                    elseif (adjusted_step_time > (full_step_time - pre_step_time))
                        first_foot_step_ratio = 1;
                    else
                        first_foot_step_ratio = (adjusted_step_time - post_step_time) / (full_step_time - post_step_time - pre_step_time);
                    end
                end
            end
            
            % Second foot
            if t < half_step_time
                second_foot_step_ratio = 0;
            elseif ~last_foot_same && t > obj.duration - half_step_time
                adjusted_step_time = t - (obj.duration - half_step_time);
                if (adjusted_step_time < post_step_time)
                    second_foot_step_ratio = 0;
                elseif (adjusted_step_time > (half_step_time - pre_step_time))
                    second_foot_step_ratio = 1;
                else
                    second_foot_step_ratio = (adjusted_step_time - post_step_time) / (half_step_time - post_step_time - pre_step_time);
                end
            else
                adjusted_step_time = t - half_step_time;
                step_num = fix(adjusted_step_time/full_step_time);
                adjusted_step_time = adjusted_step_time - step_num * full_step_time;
                if (mod(step_num, 2) == 1)
                    second_foot_step_ratio = 0;
                else
                    if (adjusted_step_time < post_step_time)
                        second_foot_step_ratio = 0;
                    elseif(adjusted_step_time > (full_step_time - pre_step_time))
                        second_foot_step_ratio = 1;
                    else
                        second_foot_step_ratio = (adjusted_step_time - post_step_time) / (full_step_time - post_step_time -pre_step_time);
                    end
                end
            end

            % Which foot is first?
            assert (first_foot_step_ratio <= 1);
            assert (second_foot_step_ratio <= 1);
            
            if obj.first_step_left
                right_foot_step_ratio = first_foot_step_ratio;
                left_foot_step_ratio = second_foot_step_ratio;
            else
                right_foot_step_ratio = second_foot_step_ratio;
                left_foot_step_ratio = first_foot_step_ratio;
            end
            step_num = step_num + 1;
        end
        
        function position = right_foot_position_at_step(obj, n)
            bodystep = getBodyStep(obj, n);
            
            bodypos = bodystep.position;
            transformToLeftFoot = Geometry.transform([0, obj.foot_separation, -bodypos(3) + obj.foot_center_to_floor]);
            position = bodystep * transformToLeftFoot;
        end
        
        function position = left_foot_position_at_step(obj, n)
            bodystep = getBodyStep(obj, n);
            
            bodypos = bodystep.position;
            transformToRightFoot = Geometry.transform([0, -obj.foot_separation, -bodypos(3) + obj.foot_center_to_floor]);
            position = bodystep * transformToRightFoot;
        end
        
        function [right_foot_action, left_foot_action] = whatIsTheFootDoing(obj, step_num)
            if step_num == 0
                if obj.first_step_left
                    right_foot_action = [0 1];   % Go from body position 0 to 1
                    left_foot_action = 0;      % Stay put at position 0
                else
                    right_foot_action = 0;
                    left_foot_action = [0 1];
                end
            elseif step_num == obj.num_steps - 1
                if xor(obj.first_step_left, mod(obj.num_steps, 2) == 0)
                    right_foot_action = [obj.num_steps-2 obj.num_steps-1];
                    left_foot_action = obj.num_steps-1;
                else
                    left_foot_action = [obj.num_steps-2 obj.num_steps-1];
                    right_foot_action = obj.num_steps-1;
                end
            else
                if (mod(step_num, 2) == 0) % Left foot moving
                    left_foot_action = step_num;
                    right_foot_action = [step_num-1 step_num+1];
                else
                    left_foot_action = [step_num-1 step_num+1];
                    right_foot_action = step_num;
                end
            end
        end
        
        function [right_foot_position, left_foot_position] = footPosition(obj, t)
            
            [step_num, right_foot_step_ratio, left_foot_step_ratio] = footHeightRatio(obj, t);
            [right_foot_action, left_foot_action] = whatIsTheFootDoing(obj, step_num);
            
            % Left foot
            if (length(right_foot_action) == 1)
                tmp = obj.right_foot_position_at_step(right_foot_action);
                right_foot_position = tmp.H;
            else
                from = obj.right_foot_position_at_step(right_foot_action(1));
                to = obj.right_foot_position_at_step(right_foot_action(2));
                
                right_foot_position = obj.parabolicPath(from, to, obj.step_height, ...
                    obj.step_outwardness, -obj.step_rotation, right_foot_step_ratio);
            end
            
            % Right foot
            if (length(left_foot_action) == 1)
                tmp = obj.left_foot_position_at_step(left_foot_action);
                left_foot_position = tmp.H;
            else
                from = obj.left_foot_position_at_step(left_foot_action(1));
                to = obj.left_foot_position_at_step(left_foot_action(2));
                
                left_foot_position = obj.parabolicPath(from, to, obj.step_height, ...
                    -obj.step_outwardness, obj.step_rotation, left_foot_step_ratio);
            end
        end
        
        function position = parabolicPath(obj, startTransform, endTransform, zdiff, sidediff, rotdiff, ratio)
            % Simple Parabolic trajectory
            % (http://mathworld.wolfram.com/ParabolicSegment.html)
            step_time = obj.bodyStepTime;
            distance_between_step = Geometry.transform.distance(startTransform, endTransform);
            height_per_step = norm([zdiff, sidediff]);
            
            h = height_per_step;
            a = distance_between_step / 2;
            
            % Evenly spaced points on parabola Method 1 - Numerical Solve
%             length_arc = sqrt(a^2 + 4*h^2) + a^2/2/h * asinh(2*h/a);
%             half_length = length_arc / 2;
%             diff_length = abs(length_so_far - half_length);
%             syms x;
%             lindist = sqrt(a^2 + h^2);
%             guess = acosh(sqrt(((abs(step_time*lindist - lindist/2))*2*h/a^2)^2 + 1));
%             lhs = diff_length*4*h/a^2;
%             tmp = double(vpasolve(lhs == x + 0.5*sinh(2*x), x, guess));
%             tmp = sqrt(cosh(tmp)^2 - 1)*a^2/2/h;
%             if (length_so_far < half_length)
%                 dist = distance_between_step/2 - tmp;
%             else
%                 dist = distance_between_step/2 + tmp;
%             end
            
            % Using Newton Approximation Method
            % https://math.stackexchange.com/questions/3129154/divide-a-parabola-in-segments-of-equal-length
            L = distance_between_step;
            aa = 4*h/L;
            
            f = @(x) x * sqrt(1+x^2) + asinh(x);
            s = ratio;
            J = @(X) 2 * sqrt(1+X^2);
            r = @(X) f(X) - (1-2*s)*f(aa);
            
            X = 0;
            while(abs(r(X)) > 0.0001)
                X = X - r(X)/J(X);
            end
            if aa == 0
                dist = ratio * L;
            else
                dist = 0.5*(1-X/aa) * L;
            end
            
            % Calculate intermediate transform
            position_time = dist / distance_between_step * step_time;
            if position_time < 0
                position_time = 0;
            end
            [t1, ~, ~] = transformtraj(startTransform.H,endTransform.H,[0 step_time],position_time);
            
            x = -a + dist;
            y = h * (1 - x^2/a^2);
            
            zdelta = cos(atan2(sidediff, zdiff)) * y;
            ydelta = sin(atan2(sidediff, zdiff)) * y;
            if rotdiff ~= 0
                thetadelta = y / height_per_step * rotdiff;
            else
                thetadelta = 0;
            end
            
            t2 = Geometry.transform([0 ydelta zdelta], axang2quat([1 0 0 thetadelta]));
            
            position = t1 * t2.H;
        end
        
        function show(obj)            
            % Draw the foot position
            i = 1;
            for t = 0:obj.step_size:obj.duration
                [lfp, rfp] = obj.footPosition(t);
                tfInterp_l(:,:,i) = lfp;
                tfInterp_r(:,:,i) = rfp;
                i = i + 1;
            end
            hold on;
            plotTransforms(tform2trvec(tfInterp_l),tform2quat(tfInterp_l), 'FrameSize', 0.01)
            plotTransforms(tform2trvec(tfInterp_r),tform2quat(tfInterp_r), 'FrameSize', 0.01)
            hold off;
        end
        
    end
end

