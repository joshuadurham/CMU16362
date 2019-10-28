classdef rotationalReferenceControl
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        wheelbase = 0.09
    end
        
    properties
        amax
        vmax
        wmax
        angleAMax
        angle
        arclen
        sgn
        tPause
        tRamp
        tf
        Tdelay
        ki
        kp
        kd
        idealArr
        delayedArr
        realArr
        errArr
        tArr
        ks
        kv
        Tf
    end
    
    methods
        function obj = rotationalReferenceControl(amax, vmax, angle, sgn, tPause)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.amax = amax;
            obj.vmax = vmax;
            obj.wmax = 2*vmax / obj.wheelbase;
            obj.angleAMax = 2*amax / obj.wheelbase;
            obj.angle = angle;
            obj.arclen = angle * obj.wheelbase / 2;
            
            obj.sgn = sgn;
            obj.tRamp = obj.wmax / obj.angleAMax;
            obj.tf = (obj.angle + ((obj.wmax).^2)/obj.angleAMax)/obj.wmax;
            obj.tPause = tPause;
            obj.ks = 1;
            obj.kv = 1;
            obj.Tf = obj.tf * obj.ks / obj.kv;
           
            % figure out what the real delay is but this should be ok
            obj.Tdelay = 0.2;
            obj.kp = 3;
            obj.ki = 0.0;
            obj.kd = 0.12;
            
            currT = 0;
            prevT = 0;

            delayedFeedForward = 0;
            currIdealAngle = 0;
            delayedCurrIdealAngle = 0;
            firstIteration = false;

            % Graphing setup
            len = 1000;
            obj.idealArr = zeros(1, len);
            obj.delayedArr = zeros(1, len);
            obj.realArr = zeros(1, len);
            obj.errArr = zeros(1, len);
            obj.tArr = zeros(1, len);

            count = 1;
            pause(3);

            while(currT < obj.getTrajectoryDuration())
                if(firstIteration == false)
                    startTic = tic();        
                    firstIteration = true;
                end
                currT = toc(startTic);
                deltaT = currT - prevT;
                [~, ff] = obj.computeControl(currT);
                feedForward = ff * sgn;
                currIdealAngle = currIdealAngle + feedForward * deltaT;
                delayedFeedForward = obj.computeControl(currT - obj.Tdelay) * sgn;
                delayedCurrIdealAngle = delayedCurrIdealAngle + delayedFeedForward * deltaT;
                obj.idealArr(count) = currIdealAngle;
                obj.delayedArr(count) = delayedCurrIdealAngle;
                obj.tArr(count) = currT;
                count = count + 1;
                prevT = currT;
                pause(0.01);
            end
            obj.idealArr = obj.idealArr(1:count-1);
            obj.delayedArr = obj.delayedArr(1:count-1);
            obj.tArr = obj.tArr(1:count-1);
        end

        function [V, w] = computeControl(obj, t)
            if (t < 0 || t > obj.tf)
                w = 0;
            elseif (t < obj.tRamp)
                w = obj.angleAMax * t;
            elseif (obj.tf - t < obj.tRamp)
                w = obj.angleAMax * (obj.tf - t);
            elseif (obj.tRamp < t && t < obj.tf - obj.tRamp)
                w = obj.wmax;
            else
                w = 0;
            end
            % computes angles turning to the left as positive, angles to
            % the right as negative
            % [~, w] = robotModel.vlvrToVw(-V, V);
            V = 0;
        end

        function duration = getTrajectoryDuration(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            duration = obj.Tf + obj.tPause;
        end
    end
end
