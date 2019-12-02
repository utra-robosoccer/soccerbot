classdef crotchpath < Geometry.footpath
    properties
        crotch_zdiff_sway = 0.000;
        crotch_sidediff_sway = -0.03;
        crotch_sidediff_sway_decay = 5;
        crotch_thetadiff_sway = [0 0 0.08];
        
        % Distort per step
        crotch_zdiff_step = 0.000;
        crotch_sidediff_step = 0.000;
        crotch_rot_step = [0 0 0.0];
        
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
            [step_num, right_foot_ratio, left_foot_ratio] = footHeightRatio(obj, t, 1);
            [right_foot_action, ~] = whatIsTheFootDoing(obj, step_num);
            if (length(right_foot_action) == 2)
                ratio = right_foot_ratio;
            else
                ratio = left_foot_ratio;
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
            
            position = obj.parabolicPath(from, to, 0.0, 0.0, 0.0, body_movement_ratio);
            
            % Vertical Sway (sinusoidal wave)
            [~, right_foot_ratio, ~] = footHeightRatio(obj, t, 3);
            if (length(right_foot_action) == 2) % Left foot moving, lean right
                ratio = right_foot_ratio;
            else
                ratio = left_foot_ratio;
            end
            if t < obj.half_step_time
                zdiff = obj.crotch_zdiff_sway * (1 - cos(ratio * pi));
            elseif t > obj.duration - obj.half_step_time
                zdiff = obj.crotch_zdiff_sway * (1 - cos(ratio * pi + pi));
            else
                zdiff = obj.crotch_zdiff_sway * (1 - cos(ratio * 2 * pi + pi));
            end
            
            % Horizontal Sway (exponential decay)
            [~, right_foot_ratio, left_foot_ratio] = footHeightRatio(obj, t, 3);
            if (length(right_foot_action) == 2)
                ratio = right_foot_ratio;
                is_right_foot = -1;
            else
                ratio = left_foot_ratio;
                is_right_foot = 1;
            end
            r = -4 * ratio^2 + 4 * ratio;
            ydiff = r * obj.crotch_sidediff_sway * is_right_foot;
            thetadiff = ydiff / obj.crotch_sidediff_sway * obj.crotch_thetadiff_sway;

            H = eul2tform(thetadiff);
            H(3,4) = zdiff;
            H(2,4) = ydiff;
            H(1,4) = 0.0;
            position = position * H;
            
%             % Step Transformations (parabola)
%             [~, right_foot_ratio, left_foot_ratio] = footHeightRatio(obj, t);
%             if (length(right_foot_action) == 2)
%                 ratio = right_foot_ratio;
%                 is_right_foot = 1;
%             else
%                 ratio = left_foot_ratio;
%                 is_right_foot = -1;
%             end
%             if (ratio ~= 0 && ratio ~= 1)
%                 r = -4 * ratio^2 + 4 * ratio;
%                 
%                 rot_step = obj.crotch_rot_step * is_right_foot * r;
%                 step_diff = eul2tform(rot_step);
%                 step_diff(3,4) = obj.crotch_zdiff_step * r;
%                 step_diff(2,4) = obj.crotch_sidediff_step * is_right_foot * r;
%                 position = position * step_diff;
%             end
            
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

