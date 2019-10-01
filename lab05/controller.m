classdef controller
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
       refPose
       acPose
       Vref
       invTwr
       diff
       rrp
       tau
       kx
       ky
       kth
    end
    
    methods
        function obj = controller(refPose,acPose,Vref,tau)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.refPose = refPose;
            obj.acPose = acPose;
            obj.Vref = abs(Vref);
            obj.invTwr = acPose.aToB();
            obj.diff = obj.refPose.getPoseVec() - obj.acPose.getPoseVec();
            obj.rrp = obj.invTwr*refPose.getPoseVec();
            obj.tau = tau;
            obj.kx = 1/obj.tau/obj.Vref;
            obj.ky = 2/(obj.tau*obj.Vref).^2;
            obj.kth = 2/obj.tau/obj.Vref;
            
        end
        
        function [uV, uOmega] = error(obj)
            ex = obj.rrp(1);
            ey = obj.rrp(2);
            eth = obj.diff(3);
            uV = obj.kx*ex;
            uOmega = obj.ky*ey+obj.kth*eth;
        end
        
    end
end