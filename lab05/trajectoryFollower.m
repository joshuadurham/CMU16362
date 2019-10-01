classdef trajectoryFollower
    %UNTITLED9 Summary of this class goes here
    %   Detailed explanation goes here
    
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
        
        function [refV, refw] = getRefVw(obj, t)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            [refV, refw] = obj.traj.getVwAtT(t);
        end
        
        function [realV, realw] = getRealVw(obj, t)
            [realV, realw] = obj.getRefVw(t);
            [uv, uw] = obj.controller.error();
            realV = realV + uv;
            realw = realw + uw;
        end
        
    end
end

