classdef robotpath < Geometry.footpath
    properties
    end
    
    methods
        function obj = robotpath(start_transform, end_transform, foot_center_to_floor)
            obj@Geometry.footpath(start_transform, end_transform, foot_center_to_floor);
        end

        function show(obj)
            show@Geometry.path(obj);
            hold on;
%             show@Geometry.crotchpath(obj);
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
                    if left_foot_step_ratio(i) == 0
                        left_foot_body_pose(i) = nan;
                    else
                        left_foot_body_pose(i) = (left_foot_action(2) - left_foot_action(1))  * left_foot_step_ratio(i) + left_foot_action(1);
                    end
                end
                
                if (length(right_foot_action) == 1)
                    right_foot_body_pose(i) = right_foot_action;
                else
                    if right_foot_step_ratio(i) == 0
                        right_foot_body_pose(i) = nan;
                    else
                        right_foot_body_pose(i) = (right_foot_action(2) - right_foot_action(1))  * right_foot_step_ratio(i) + right_foot_action(1);
                    end
                end
                
                i = i + 1;
            end
            
            subplot(2,1,1);
            plot(times, left_foot_step_ratio);
            hold on;
            plot(times, right_foot_step_ratio);
            title('Foot step Ratio');
            xlabel('time (t)');
            ylabel('Ratio');
            grid off;
            grid minor;
            legend('Left', 'Right');
            
            subplot(2,1,2);
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

        end
    end
end

