classdef Transform < handle
    %Transform a general position on the field
    
    properties
        % Position
        position = [0,0,0];

        % In Quaternion Format
        orientation = [1,0,0,0];
    end
    
    methods
        function obj = Transform(position, orientation)
            obj.position = position;
            if nargin == 2
                obj.orientation = orientation;
            end
        end
        
        function objout = ApplyTransformation(obj, t2)
            Hnew = (t2.H) * (obj.H);
            objout = Geometry.Transform([Hnew(1,4), Hnew(2,4), Hnew(3,4)], rotm2quat(Hnew(1:3, 1:3)));
        end
        
        function H = H(obj)
            H = quat2tform(obj.orientation);
            H(1,4) = obj.position(1);
            H(2,4) = obj.position(2);
            H(3,4) = obj.position(3);
        end
        
        function X = X(obj)
            X = obj.position(1);
        end
        
        function Y = Y(obj)
            Y = obj.position(2);
        end
        
        function Z = Z(obj)
            Z = obj.position(3);
        end
        
        function [r, p, y] = AxAngles(obj)
            axang = quat2axang(obj.orientation);
            r = axang(1);
            p = axang(2);
            y = axang(3);
        end
        
        function roll = Roll(obj)
            [roll, ~, ~] = obj.AxAngles;
        end
        
        function pitch = Pitch(obj)
            [~, pitch, ~] = obj.AxAngles;
        end
        
        function yaw = Yaw(obj)
            [~, ~, yaw] = obj.AxAngles;
        end
        
        function Draw(obj, label)
            if (nargin < 2)
                label = 'tform';
            end
            
            trplot(obj.H, 'frame', label, 'length', 0.05)
        end
    end
    
    methods(Abstract)
        angles = InverseKinematics(obj)
    end
end