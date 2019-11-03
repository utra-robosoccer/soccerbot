classdef path < handle
    properties
        step_size = 0.05; % Not absolutely fixed, will be modified slightly when
        speed = 0.05;
        turn_duration = 4; % Number of steps to turn
        start_transform;
        end_transform;
        
        % Computed numbers
        distance;
        distanceMap;
    end
    
    methods
        function obj = path(start_transform, end_transform)
            obj.start_transform = start_transform;
            obj.end_transform = end_transform;
            
            % Compute approximate distance and distance map
            precision = 0.05 * obj.step_size;
            precisions = precision:precision:1;
            obj.distance = 0;
            prev_pose = obj.positionAtRatio(0);
            
            obj.distanceMap = zeros(length(precisions), 2);
            j = 1;
            for i = precisions
                new_pose = obj.positionAtRatio(i);
                obj.distance = obj.distance + Geometry.transform.distance(prev_pose, new_pose);
                prev_pose = new_pose;
                
                obj.distanceMap(j,1:2) = [i, obj.distance];
                j = j + 1;
            end
            
            % Round to nearest step
            s_count = obj.stepCount;
            if mod(obj.distance, obj.step_size) < obj.step_size / 2
                obj.step_size = obj.distance / s_count;
            else
                obj.step_size = obj.distance / (s_count + 1);
            end
        end
        
        function step = getStep(obj, n)
            [~, idx] = min(abs((n * obj.step_size) - obj.distanceMap(:,2)));
            step = obj.positionAtRatio(obj.distanceMap(idx, 1));
        end
        
        function stepCount = stepCount(obj)
            stepCount = fix(obj.distance / obj.step_size);
        end
        
        function duration = duration(obj)
            duration = obj.speed * obj.distance;
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
            for i = 0:1:obj.stepCount
                step = obj.getStep(i);
                position(i+1,1:3) = step.position;
                orientation(i+1,1:3) = step.H(1:3,1:3) * [0.005; 0; 0];
            end
            
            quiver3(position(:,1), position(:,2), position(:,3), orientation(:,1), orientation(:,2), orientation(:,3), 'AutoScaleFactor', 0.1);
            zlim([0,0.5]);
        end
    end
end

