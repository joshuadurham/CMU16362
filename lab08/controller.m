classdef controller
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    properties
       refPose
       acPose
       Vref
       invTwr
       Twp
       diff
       Trp
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
            obj.Twp = refPose.bToA();
            obj.Trp = obj.invTwr*obj.Twp;
            obj.tau = tau;
            obj.kx = 1/obj.tau/obj.Vref;
            obj.ky = 2/(obj.tau*obj.Vref).^2;
            obj.kth = 2/obj.tau/obj.Vref;
        end
        
        function [uV, uOmega] = error(obj)
            errVec = pose.matToPoseVec(obj.Trp);
            ex = errVec(1);
            ey = errVec(2);
            eth = errVec(3);
            % not sure if next line is necessary but hey why not
            eth = atan2(sin(eth), cos(eth));
            uV = obj.kx*ex;
            thLimit = 10;
            if eth > thLimit 
                eth = thLimit;
            elseif eth < -thLimit 
                eth = -thLimit;
            end
            uOmega = obj.ky*ey+obj.kth*eth;
        end
        
        function realPose = getRealPose(obj)
            realPose = obj.acPose;
        end
        
    end
end