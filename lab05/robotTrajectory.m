classdef robotTrajectory
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       ref
       poseSamples
       vlSamples
       vrSamples
       tSamples
       wSamples
       VSamples
       length
       startx
       starty
       startth
       starts
    end
    
    methods
        function obj = robotTrajectory(ref, startx, starty, startth, starts, numSamples, startTime, endTime)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.ref = ref;
            obj.startx = startx;
            obj.starty = starty;
            obj.startth = startth;
            obj.starts = starts;
        
            obj.tSamples = linspace(startTime, endTime, numSamples);
            obj.length = size(obj.tSamples, 2);
            
            obj.poseSamples = zeros(3, obj.length);
            obj.vlSamples = zeros(1, obj.length);
            obj.vrSamples = zeros(1, obj.length);
            obj.VSamples = zeros(1, obj.length);
            obj.wSamples = zeros(1, obj.length);
            
            slast = obj.starts; % maybe need a starting pose
            tlast = startTime;
            count = 1;
            
            x = obj.startx;
            y = obj.starty;
            th = obj.startth;
            
            for i=1:obj.length
                t = obj.tSamples(i);
                T = t * obj.ref.ks / obj.ref.kv;
                if ((T < obj.ref.tPause) || (T > obj.ref.tPause + obj.ref.Tf))
                    vl = 0;
                    vr = 0;
                    v = 0;
                    w = 0;
                    s = slast;
                else
                    [v, w] = obj.ref.computeControl(t);
                    dt = t - tlast;
                    
                    s = v .* dt + slast;
                    % s = v .* (t - obj.ref.tPause / (obj.ref.ks / obj.ref.kv));
                    % s = v .* dt + slast;
                    s = v .* (t - obj.ref.tPause / (obj.ref.ks / obj.ref.kv));
                    ds = s - slast;

                    [vl, vr] = robotModel.VwTovlvr(v, w);
                    thReal = thReal + t.ref.ks * w ./2 * dt;
                    xReal = xReal + v * 1.1458 * dt * cos(thReal);
                    yReal = yReal + v * 1.1458 * dt * sin(thReal);
                    thReal = thReal +  t.ref.ks * w ./2 * dt;
                    th = th + obj.ref.ks * w ./ 2 * dt;

                    x = x + obj.ref.ks * ds * cos(th);
                    y = y + obj.ref.ks * ds * sin(th);

                    x = x + ds * obj.ref.ks * cos(th);
                    y = y + ds * obj.ref.ks * sin(th);
                    th = th + obj.ref.ks * w ./ 2 * dt;
                end
                
                pose = [x; y; th];
                obj.vlSamples(count) = vl;
                obj.vrSamples(count) = vr;
                obj.poseSamples(:, count) = pose;
                obj.tSamples(count) = t;
                obj.wSamples(count) = w;
                obj.VSamples(count) = v;

                % Update for next iter
                count = count + 1;
                tlast = t;
                slast = s;
            end
        end
        
        function [vl, vr] = getVlVrAtT(obj, t)
            vl = interp1(obj.tSamples, obj.vlSamples, t);
            vr = interp1(obj.tSamples, obj.vrSamples, t);
        end
        
        function [V, w] = getVwAtT(obj, t)
            V = interp1(obj.tSamples, obj.VSamples, t);
            w = interp1(obj.tSamples, obj.wSamples, t);
        end
            
        function [xval, yval, thval] = getPoseAtT(obj,t)
            if (t > obj.ref.tf + 2 * obj.ref.tPause)
                xval = 0;
                yval = 0;
                thval = 0;
                return;
            end
            x = obj.poseSamples(1, :);
            y = obj.poseSamples(2, :);
            th = obj.poseSamples(3, :);
            xval = interp1(obj.tSamples, x, t);
            yval = interp1(obj.tSamples, y, t);
            thval = interp1(obj.tSamples, th, t);
        end
        
    end
end

