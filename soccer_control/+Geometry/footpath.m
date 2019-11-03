classdef footpath < Geometry.path
    properties
        foot_separation = 0.05;
    end
    
    methods
        function obj = footpath(start_transform, end_transform)
            obj = obj@Geometry.path(start_transform, end_transform);
        end
    end
end

