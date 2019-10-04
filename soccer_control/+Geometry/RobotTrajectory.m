classdef RobotTrajectory
    properties
        robotTransformList = Geometry.RobotTransform.empty; % Array of robot transforms
    end
    
    methods
        function obj = RobotTrajectory(robotTransformList)
            obj.robotTransformList = transformList;
        end
    end
end

