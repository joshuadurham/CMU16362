classdef trajectoryFollower
    
    properties
        traj
        controller
    end
    
    methods
        function obj = trajectoryFollower(traj, controller)
            %UNTITLED9 Construct an instance of this class
            %   Detailed explanation goes here
            obj.traj = traj;
            obj.controller = controller;
        end
        
        function [refV, refw] = getRefVwAtT(obj, t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            refV = obj.traj.getVAtTime(t);
            refw = obj.traj.getwAtTime(t);
        end
        
        function refPose = getRefPoseAtT(obj, t)
            
        
        function [realV, realw] = getRealVwAtT(obj, t)
            [realV, realw] = obj.getRefVw(t);
            if (isnan(realV))
                realV = 0;
            end
            if (isnan(realw))
                realw = 0;
            end
            [uv, uw] = obj.controller.error();
            realV = realV + uv;
            realw = realw + uw;
        end
        
    end
end

