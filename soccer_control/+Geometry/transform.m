classdef transform < handle
    %Transform a general position on the field
    
    properties
        % H Matrix
        H = [1 0 0 0; 
             0 1 0 0; 
             0 0 1 0; 
             0 0 0 1];
    end
    
    methods
        function obj = transform(arg1, arg2, arg3, arg4)
            if nargin == 1
                if (size(arg1) == 4)
                    obj.H = arg1;
                else
                    obj.setPosition(arg1);
                end
            elseif nargin == 2
                obj.setPosition(arg1);
                obj.setOrientation(arg2);
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
        
        function setOrientation(obj, rot)
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
    methods(Static)
        function distance = distance(p1, p2)
            distance = norm(p1.H(1:3,4) - p2.H(1:3,4));
        end
    end
end