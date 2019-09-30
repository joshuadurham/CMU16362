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
            obj.tf = (obj.dist + ((obj.vmax).^2)/obj.amax)/obj.vmax;
            obj.tPause = tPause;
            obj.ks = 1;
            obj.kv = 1;
            obj.Tf = obj.tf * obj.ks / obj.kv;
           
            
            obj.Tdelay = 0.2;
            obj.kp = 3;
            obj.ki = 0.0;
            obj.kd = 0.12;
            
            % useFeedback = false;
            currT = 0;
            prevT = 0;

            feedForward = 0;
            delayedFeedForward = 0;
            currIdealPos = 0;
            delayedCurrIdealPos = 0;
            firstIteration = false;

            % Graphing setup
            len = 1000;
            obj.idealArr = zeros(1, len);
            obj.delayedArr = zeros(1, len);
            obj.realArr = zeros(1, len);
            obj.errArr = zeros(1, len);
            obj.tArr = zeros(1, len);

            count = 1;
            % realX = x;
%             error = currIdealPos - realX;
            pause(3);

            while(currT < 6)
                if(firstIteration == false)
                    startTic = tic();        
                    firstIteration = true;
                end
                currT = toc(startTic);
                deltaT = currT - prevT;
                currIdealPos = currIdealPos + feedForward * deltaT;
                delayedCurrIdealPos = delayedCurrIdealPos + delayedFeedForward * deltaT;
                % error = delayedCurrIdealPos - realX;
                % errorDelta = (error - prevError)/deltaT;
                % accError = accError + error*deltaT;
                % cmd = kp*error + ki*accError + kd*errorDelta;
%                 if (cmd > 0.3)
%                     cmd = 0.3;
%                 end
                feedForward = obj.computeControl(currT);
                delayedFeedForward = obj.computeControl(currT - obj.Tdelay);
%                 if (useFeedback) 
%                     robot.sendVelocity(cmd + feedForward, cmd + feedForward);
%                 else
%                     robot.sendVelocity(feedForward, feedForward);
%                 end
                prevT = currT;
%                prevError = error;
                obj.idealArr(count) = currIdealPos;
%                 obj.realArr(count) = realX;
                obj.delayedArr(count) = delayedCurrIdealPos;
%                obj.errArr(count) = error;
                obj.tArr(count) = currT;
                count = count + 1;
%                 realX = x;
                pause(0.01);
            end
            obj.idealArr = obj.idealArr(1:count-1);
            obj.delayedArr = obj.delayedArr(1:count-1);
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

        function duration = getTrajectoryDuration(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            duration = obj.tF + 2 * obj.tPause;
        end
    end
end

    
