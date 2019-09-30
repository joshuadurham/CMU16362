classdef controller
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       refPose
       acPose
    end
    
    methods
        function obj = controller(refPose,acPose)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.refPose = refPose;
            obj.acPose = acPose;
            obj.rrp = acPose.bToA(refPose);
            
        end
        
        function [V, omega] = error(obj)
            vl = interp1(obj.tSamples, obj.vlSamples, t);
            vr = interp1(obj.tSamples, obj.vrSamples, t);
        end
        
    end
end