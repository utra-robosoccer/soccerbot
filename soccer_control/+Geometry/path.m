classdef path < handle
    properties
        bodystep_size = 0.01;   % Not absolutely fixed, will be modified slightly when
        speed = 0.01;
        turn_duration = 4;      % Number of body steps to turn
        step_size = 0.03;        % Time for a single step
        
        pre_footstep_ratio = 0.1;   % Ratio of fullstep duration to keep foot on ground on prefootstep
        post_footstep_ratio = 0.3;  % Ratio of fullstep duration to keep foot on ground on postfootstep
        
        start_transform;
        end_transform;
        
        % Computed numbers
        distance;
        distanceMap;
    end
    
    methods
        function obj = path(start_transform, end_transform)
            
            if nargin == 0
                start_transform = Geometry.transform([0 0 0]);
                end_transform = Geometry.transform([0 0 0]);
            end
            
            obj.start_transform = start_transform;
            obj.end_transform = end_transform;
            
            % Compute approximate distance and distance map
            precision = 0.05 * obj.bodystep_size;
            precisions = precision:precision:1;
            obj.distance = 0;
            prev_pose = obj.positionAtRatio(0);
            
            obj.distanceMap = zeros(length(precisions), 2);
            obj.distanceMap(1,1:2) = [0, 0];
            j = 2;
            for i = precisions
                new_pose = obj.positionAtRatio(i);
                obj.distance = obj.distance + Geometry.transform.distance(prev_pose, new_pose);
                prev_pose = new_pose;
                
                obj.distanceMap(j,1:2) = [i, obj.distance];
                j = j + 1;
            end
            
            % Round to nearest step
            s_count = obj.bodyStepCount;
            if mod(obj.distance, obj.bodystep_size) < obj.bodystep_size / 2
                obj.bodystep_size = obj.distance / s_count;
            else
                obj.bodystep_size = obj.distance / (s_count + 1);
            end
        end
        
        function bodystep = getBodyStep(obj, n)
            [~, idx] = min(abs((n * obj.bodystep_size) - obj.distanceMap(:,2)));
            
            bodystep = obj.positionAtRatio(obj.distanceMap(idx, 1));
        end
        
        function bodyStepCount = bodyStepCount(obj)
            bodyStepCount = fix(obj.distance / obj.bodystep_size);
        end
        
        function duration = duration(obj)
            duration = obj.distance / obj.speed;
        end
        
        function bodyStepTime = bodyStepTime(obj)
            bodyStepTime = obj.duration / obj.bodyStepCount;
        end
        
        function pose = positionAtRatio(obj, t)
            pose = obj.poseAtRatio(t);
            pose_del = obj.poseAtRatio(t + 0.001);
            del_pose = pose_del.position - pose.position;
            
            del_theta = atan2(del_pose(2), del_pose(1));
            del_psi = atan2(del_pose(3), norm(del_pose(1:2)));
            
            orientation = eul2quat([del_theta -del_psi 0]);
            pose.setOrientation(orientation);
        end

        function pose = poseAtRatio(obj, t)
            p1 = obj.start_transform;
            p2 = obj.start_transform * Geometry.transform([obj.speed * obj.turn_duration 0 0]);
            p3 = obj.end_transform * Geometry.transform([-obj.speed * obj.turn_duration 0 0]);
            p4 = obj.end_transform;
            
            p1_pos = p1.position;
            p2_pos = p2.position;
            p3_pos = p3.position;
            p4_pos = p4.position;
            
            position = [0 0 0];
            for d = 1:3
                bez_param = [p1_pos(d) p2_pos(d) p3_pos(d) p4_pos(d)];
                
                % Cubic bezier
                for i = 0:3
                    position(d) = position(d) + bez_param(i+1) * nchoosek(3, i) * (1 - t)^(3 - i) * t ^ i;
                end
            end
            pose = Geometry.transform(position);
        end
        
        function show(obj)
            for i = 0:1:obj.bodyStepCount
                step = obj.getBodyStep(i);
                position(i+1,1:3) = step.position;
                orientation(i+1,1:3) = step.H(1:3,1:3) * [0.005; 0; 0];
            end
            
            quiver3(position(:,1), position(:,2), position(:,3), orientation(:,1), orientation(:,2), orientation(:,3), 'AutoScaleFactor', 0.1);
            zlim([0,0.5]);
        end
    end
end

