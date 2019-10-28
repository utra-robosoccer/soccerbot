classdef transform < handle
    %Transform a general position on the field
    
    properties
        % Position
        position = [0,0,0];

        % In Quaternion Format
        orientation = [1,0,0,0];
    end
    
    methods
        function obj = transform(position, orientation)
            if nargin >= 2
                obj.position = position;
            end
            if nargin >= 2
                obj.orientation = orientation;
            end
        end
        
        function objout = mtimes(obj, t2)
            Hnew = (t2.H) * (obj.H);
            objout = Geometry.Transform([Hnew(1,4), Hnew(2,4), Hnew(3,4)], rotm2quat(Hnew(1:3, 1:3)));
        end
        
        function H = H(obj)
            H = quat2tform(obj.orientation);
            H(1,4) = obj.position(1);
            H(2,4) = obj.position(2);
            H(3,4) = obj.position(3);
        end
        
        function rpy = rpy(obj)
            rpy = quat2eul(obj.orientation);
        end
    end
end