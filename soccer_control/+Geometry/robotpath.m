classdef robotpath < Geometry.crotchpath
    % Nice video for reference
    % https://www.youtube.com/watch?v=vq9A5FD8G5w
    
    properties
    end
    
    methods
        function obj = robotpath(start_transform, end_transform, foot_center_to_floor)
            obj@Geometry.crotchpath(start_transform, end_transform, foot_center_to_floor);
        end

        function show(obj)
            show@Geometry.path(obj);
            hold on;
            show@Geometry.crotchpath(obj);
            show@Geometry.footpath(obj);
            hold off;
        end
        
        function showTimingDiagram(obj)
            
            times = 0:obj.step_size:obj.duration;
            i = 1;
            for t = times
                
                [step_num(i), left_foot_step_ratio(i), right_foot_step_ratio(i)] = obj.footHeightRatio(t);
                [left_foot_action, right_foot_action] = whatIsTheFootDoing(obj, step_num(i));
                if (length(left_foot_action) == 1)
                    left_foot_body_pose(i) = left_foot_action;
                else
                    left_foot_body_pose(i) = (left_foot_action(2) - left_foot_action(1))  * left_foot_step_ratio(i) + left_foot_action(1);
                end
                
                if (length(right_foot_action) == 1)
                    right_foot_body_pose(i) = right_foot_action;
                else
                    right_foot_body_pose(i) = (right_foot_action(2) - right_foot_action(1))  * right_foot_step_ratio(i) + right_foot_action(1);
                end
                
                i = i + 1;
            end
            
            % Foot Step ratio
            subplot(3,2,1);
            plot(times, left_foot_step_ratio);
            hold on;
            plot(times, right_foot_step_ratio);
            title('Foot step Ratio');
            xlabel('time (t)');
            ylabel('Ratio');
            grid off;
            grid minor;
            legend('Left', 'Right');
            
            % Foot step
            subplot(3,2,3);
            plot(times, step_num);
            hold on;
            plot(times, left_foot_body_pose);
            plot(times, right_foot_body_pose);
            title('Foot body pose');
            xlabel('time (t)');
            ylabel('Body Pose');
            grid off;
            grid minor;
            legend('Step Num', 'Left', 'Right');
            
            % Music
            times = 0:obj.step_size:obj.duration;
            i = 1;
            for t = times
                [lfp(:,:,i), rfp(:,:,i)] = obj.footPosition(t);
                crp(:,:,i) = obj.crotchPosition(t);
                diff_left_foot(:,:,i) = lfp(:,:,i) / crp(:,:,i);
                diff_right_foot(:,:,i) = rfp(:,:,i) / crp(:,:,i);
                i = i + 1;
            end
            
            subplot(3,2,2);
            plot(times, squeeze(lfp(1,4,:)));
            hold on;
            plot(times, squeeze(rfp(1,4,:)));
            plot(times, squeeze(crp(1,4,:)));
            
            title('X position of left, right and body');
            xlabel('time (t)');
            ylabel('Torso to feet (x)');
            grid off;
            grid minor;
            legend('Left','Right', 'Crotch');
            
            subplot(3,2,5);
            plot(times, vecnorm(squeeze(diff_left_foot(1:3,4,:))));
            hold on;
            plot(times, vecnorm(squeeze(diff_right_foot(1:3,4,:))));
            title('Absolute distance between torso and foot');
            xlabel('time (t)');
            ylabel('Torso to feet (abs)');
            grid off;
            grid minor;
            legend('Left','Right');
            
            subplot(3,2,4);
            plot(times, squeeze(diff_left_foot(2,4,:)));
            hold on;
            plot(times, squeeze(diff_right_foot(2,4,:)));
            title('Diff between feet and body');
            xlabel('time (t)');
            ylabel('Torso to feet (y)');
            grid off;
            grid minor;
            legend('Left','Right');

            subplot(3,2,6);
            plot(times, squeeze(crp(3,4,:)));
            hold on;
            plot(times, squeeze(lfp(3,4,:)));
            plot(times, squeeze(rfp(3,4,:)));
            title('Z position of left, right and body');
            xlabel('time (t)');
            ylabel('Torso to feet (z)');
            grid off;
            grid minor;
            legend('Crotch', 'Left','Right');

        end
    end
end

