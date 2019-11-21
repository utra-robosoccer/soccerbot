classdef footpath < Geometry.crotchpath
    properties
        half_to_full_step_time_ratio = 0.7; % Duration difference between half and full step
        
        foot_separation = 0.035;     % seperation between feet and center
        step_height = 0.015;        % height of step
        step_outwardness = 0.0;
        step_rotation = 0.0;
        
        foot_center_to_floor;
    end
    
    methods
        function obj = footpath(start_transform, end_transform, foot_center_to_floor)
            obj@Geometry.crotchpath(start_transform, end_transform);
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
        
        function [step_num, left_foot_step_ratio, right_foot_step_ratio] = footHeightRatio(obj, t)
            full_step_time = obj.full_step_time;
            half_step_time = obj.half_step_time;
            
            post_step_time = obj.post_footstep_ratio * full_step_time;
            pre_step_time = obj.pre_footstep_ratio * full_step_time;
            
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
                left_foot_step_ratio = first_foot_step_ratio;
                right_foot_step_ratio = second_foot_step_ratio;
            else
                left_foot_step_ratio = second_foot_step_ratio;
                right_foot_step_ratio = first_foot_step_ratio;
            end
            step_num = step_num + 1;
        end
        
        function position = left_foot_position_at_step(obj, n)
            bodystep = getBodyStep(obj, n);
            
            bodypos = bodystep.position;
            transformToLeftFoot = Geometry.transform([0, -obj.foot_separation, -bodypos(3) + obj.foot_center_to_floor]);
            position = bodystep * transformToLeftFoot;
        end
        
        function position = right_foot_position_at_step(obj, n)
            bodystep = getBodyStep(obj, n);
            
            bodypos = bodystep.position;
            transformToRightFoot = Geometry.transform([0, obj.foot_separation, -bodypos(3) + obj.foot_center_to_floor]);
            position = bodystep * transformToRightFoot;
        end
        
        function [left_foot_action, right_foot_action] = whatIsTheFootDoing(obj, step_num)
            if step_num == 0
                if obj.first_step_left
                    left_foot_action = [0 1];   % Go from body position 0 to 1
                    right_foot_action = 0;      % Stay put at position 0
                else
                    left_foot_action = 0;
                    right_foot_action = [0 1];
                end
            elseif step_num == obj.num_steps - 1
                if xor(obj.first_step_left, mod(obj.num_steps, 2) == 0)
                    left_foot_action = [obj.num_steps-2 obj.num_steps-1];
                    right_foot_action = obj.num_steps-1;
                else
                    right_foot_action = [obj.num_steps-2 obj.num_steps-1];
                    left_foot_action = obj.num_steps-1;
                end
            else
                if (mod(step_num, 2) == 0) % Left foot moving
                    right_foot_action = step_num;
                    left_foot_action = [step_num-1 step_num+1];
                else
                    right_foot_action = [step_num-1 step_num+1];
                    left_foot_action = step_num;
                end
            end
        end
        
        function [left_foot_position, right_foot_position] = footPosition(obj, t)
            if (abs(t - obj.duration) < 1e-5)
                left_foot_position = obj.end_transform.H;
                right_foot_position = obj.end_transform.H;
                return;
            end
            
            [step_num, left_foot_step_ratio, right_foot_step_ratio] = footHeightRatio(obj, t);
            
            [left_foot_action, right_foot_action] = whatIsTheFootDoing(obj, step_num);
            
            % Left foot
            if (length(left_foot_action) == 1)
                tmp = obj.left_foot_position_at_step(left_foot_action);
                left_foot_position = tmp.H;
            else
                from = obj.left_foot_position_at_step(left_foot_action(1));
                to = obj.left_foot_position_at_step(left_foot_action(2));
                
                left_foot_position = obj.parabolicPath(from, to, obj.step_height, ...
                    -obj.step_outwardness, -obj.step_rotation, left_foot_step_ratio);
            end
            
            % Right foot
            if (length(right_foot_action) == 1)
                tmp = obj.right_foot_position_at_step(right_foot_action);
                right_foot_position = tmp.H;
            else
                from = obj.right_foot_position_at_step(right_foot_action(1));
                to = obj.right_foot_position_at_step(right_foot_action(2));
                
                right_foot_position = obj.parabolicPath(from, to, obj.step_height, ...
                    obj.step_outwardness, obj.step_rotation, right_foot_step_ratio);
            end
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

