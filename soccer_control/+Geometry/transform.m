classdef transform < handle
    %Transform a general position on the field
    
    properties
        % H Matrix
        H = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    end
    
    methods
        function obj = transform(arg1, arg2, arg3, arg4)
            if nargin == 1
                setPosition(arg1);
            elseif nargin == 2
                setPosition(arg1);
                setRotation(arg2);
            elseif nargin == 4
                a = arg1;
                alpha = arg2;
                d = arg3;
                theta = arg4;
                
                obj.H = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta); ...
                    sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta); ...
                    0 sin(alpha) cos(alpha) d; ...
                    0 0 0 1];
            end
        end
        
        function objout = mtimes(obj, t2)
            objout = Geometry.transform();
            objout.H = obj.H * t2.H;
        end
        
        function quat = orientation(obj)
            quat = tform2quat(obj.H);
        end
        
        function setRotation(obj, rot)
            obj.H(1:3,1:3) = quat2rotm(rot);
        end
        
        function pos = position(obj)
            pos = obj.H(1:3,4)';
        end
        
        function setPosition(obj, pos)
            obj.H(1:3,4) = pos';
        end
        
        function rpy = rpy(obj)
            rpy = quat2eul(obj.orientation);
        end
    end
end