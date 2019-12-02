classdef trapezoidalReferenceControl
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        amax
        vmax
        dist
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
        velArr
        tArr
        ks
        kv
        Tf
    end
    
    methods
        function obj = trapezoidalReferenceControl(amax, vmax, dist, sgn, tPause)
            %UNTITLED3 Construct an instance of this class
            %   Detailed explanation goes here
            obj.amax = amax;
            obj.vmax = vmax;
            obj.dist = dist;
            obj.sgn = sgn;
            obj.tRamp = obj.vmax / obj.amax;
            
            if obj.dist < obj.vmax * obj.tRamp
                obj.vmax = obj.dist / 2;
                obj.amax = obj.vmax;
                obj.tRamp = obj.vmax / obj.amax;
            end 
            obj.tf = (obj.dist + ((obj.vmax).^2)/obj.amax)/obj.vmax;
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

            currIdealPos = 0;
            delayedCurrIdealPos = 0;
            firstIteration = false;

            % Graphing setup
            len = 1000;
            obj.idealArr = zeros(1, len);
            obj.delayedArr = zeros(1, len);
            obj.realArr = zeros(1, len);
            obj.errArr = zeros(1, len);
            obj.velArr = zeros(1, len);
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
                feedForward = obj.computeControl(currT);
                obj.velArr(count) = feedForward;
                currIdealPos = currIdealPos + feedForward * deltaT;
                delayedFeedForward = obj.computeControl(currT - obj.Tdelay);
                delayedCurrIdealPos = delayedCurrIdealPos + delayedFeedForward * deltaT;
                obj.idealArr(count) = currIdealPos;
                obj.delayedArr(count) = delayedCurrIdealPos;
                obj.tArr(count) = currT;
                count = count + 1;
                prevT = currT;
                pause(0.01);
            end
            obj.idealArr = obj.idealArr(1:count-1);
            obj.delayedArr = obj.delayedArr(1:count-1);
            obj.velArr = obj.velArr(1:count-1);
            obj.tArr = obj.tArr(1:count-1);
        end

        function [V, w] = computeControl(obj, t)
            w = 0;
            if (t < 0 || t > obj.tf)
                V = 0;
            elseif (t < obj.tRamp)
                V = obj.amax * t;
            elseif (obj.tf - t < obj.tRamp)
                V = obj.amax * (obj.tf - t);
            elseif (obj.tRamp < t && t < obj.tf - obj.tRamp)
                V = obj.vmax;
            else
                V = 0;
            end
            V = obj.sgn * V;
        end
        
        function V = getVAtTime(obj, t)
            V = interp1(obj.tArr, obj.velArr, t);
            if isnan(V)
                V = 0;
            end
        end
              
        function pose = getPoseAtTime(obj, t)
            pose = [0, 0, 0];
            pose(1) = interp1(obj.tArr, obj.idealArr, t);
        end
        
        function w = getwAtTime(obj, t)
            w = 0;
        end

        function duration = getTrajectoryDuration(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            duration = obj.Tf + obj.tPause;
        end
    end
end

    
