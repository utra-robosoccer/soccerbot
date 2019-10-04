classdef RobotTransform
    properties
        worldToBodyTransform
        bodyToLeftFootTransform
        bodyToRightFootTransform
        bodyToLeftArmTransform
        bodyToRightArmTransform
    end
    
    methods
        function obj = RobotTransform(worldToBodyTransform, bodyToLeftFootTransform,...
                bodyToRightFootTransform, bodyToLeftArmTransform, bodyToRightArmTransform)
            
            obj.worldToBodyTransform     = worldToBodyTransform;
            obj.bodyToLeftFootTransform  = bodyToLeftFootTransform;
            obj.bodyToRightFootTransform = bodyToRightFootTransform;
            obj.bodyToLeftArmTransform   = bodyToLeftArmTransform;
            obj.bodyToRightArmTransform  = bodyToRightArmTransform;
        end
        
        function q = InverseKinematics(dh, x6, y6, z6, g, q0)
            
        end
        
    end
end

