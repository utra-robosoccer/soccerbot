classdef crotchpath < Geometry.footpath
    properties
        crotch_zdiff_per_step = -0.01;
        crotch_sidediff_step = 0.01;
        crotch_rotation_per_step = 0.0;
        
        sideways_exponential_decay_rate = 5;
        
        first_step_left = 0;
    end
    
    methods
        function obj = crotchpath(start_transform, end_transform, foot_center_to_floor)
            obj = obj@Geometry.footpath(start_transform, end_transform, foot_center_to_floor);
            
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
            [step_num, left_foot_ratio, right_foot_ratio] = footHeightRatio(obj, t, 1);
            [left_foot_action, ~] = whatIsTheFootDoing(obj, step_num);
            if (length(left_foot_action) == 2)
                ratio = left_foot_ratio;
            else
                ratio = right_foot_ratio;
            end

            % Base position for the torso
            if step_num == 0
                from = obj.getBodyStep(0);
                to = obj.getBodyStep(1);
                body_movement_ratio = ratio / 2;
            elseif step_num == obj.num_steps - 1
                from = obj.getBodyStep(step_num - 1);
                to = obj.getBodyStep(step_num);
                body_movement_ratio = ratio / 2 + 1/2;
            else                
                if (ratio < 0.5)
                    from = obj.getBodyStep(step_num - 1);
                    to = obj.getBodyStep(step_num);
                    body_movement_ratio = ratio + 0.5;
                else
                    from = obj.getBodyStep(step_num);
                    to = obj.getBodyStep(step_num + 1);
                    body_movement_ratio = ratio - 0.5;
                end
            end
            
            position = obj.parabolicPath(from, to, 0, 0, 0, body_movement_ratio);
            
            % Add horizontal delta (exponential decay)
            [~, left_foot_ratio, ~] = footHeightRatio(obj, t, 2);
            if (length(left_foot_action) == 2) % Left foot moving, lean right
                ydiff = obj.crotch_sidediff_step * (1 - exp(-obj.sideways_exponential_decay_rate * left_foot_ratio));
            else
                ydiff = -obj.crotch_sidediff_step * (1 - exp(-obj.sideways_exponential_decay_rate * right_foot_ratio));
            end
            
            % Add vertical delta (sinusoidal wave)
            [~, left_foot_ratio, ~] = footHeightRatio(obj, t, 3);
            if (length(left_foot_action) == 2) % Left foot moving, lean right
                ratio = left_foot_ratio;
            else
                ratio = right_foot_ratio;
            end
            if t < obj.half_step_time
                zdiff = obj.crotch_zdiff_per_step * (1 - cos(ratio * pi));
            elseif t > obj.duration - obj.half_step_time
                zdiff = obj.crotch_zdiff_per_step * (1 - cos(ratio * pi + pi));
            else
                zdiff = obj.crotch_zdiff_per_step * (1 - cos(ratio * 2 * pi + pi));
            end
            
            H = Geometry.transform([0 ydiff zdiff]);
            position = position * H.H;
        end
                
        function show(obj)            
            % Draw the crotch position
            i = 1;
            for t = 0:obj.step_size:obj.duration
                tfInterp(:,:,i) = obj.crotchPosition(t);
                i = i + 1;
            end
            hold on;
            plotTransforms(tform2trvec(tfInterp),tform2quat(tfInterp), 'FrameSize', 0.01)
            hold off;
        end
    end
end

