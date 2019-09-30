classdef robotTrajectory
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
       ref
       poseSamples
       vlSamples
       vrSamples
       tSamples
       length
       startx
       starty
    end
    
    methods
        function obj = robotTrajectory(ref, startx, starty, numSamples)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj.ref = ref;
            obj.startx = startx;
            obj.starty = starty;
        
            obj.tSamples = linspace(0, obj.ref.tf + obj.ref.tPause * 2, numSamples);
            obj.length = size(obj.tSamples, 2);
            
            obj.poseSamples = zeros(3, obj.length);
            obj.vlSamples = zeros(obj.length, 1);
            obj.vrSamples = zeros(obj.length, 1);
            
            slast = 0; % maybe need a starting pose
            tlast = 0;
            count = 1;
            
            x = obj.startx;
            y = obj.starty;
            th = 0;
            
            for i=1:obj.length
                t = obj.tSamples(i);
                T = t * obj.ref.ks / obj.ref.kv;
                if ((T < obj.ref.tPause) || (T > obj.ref.tPause + obj.ref.Tf))
                    vl = 0;
                    vr = 0;
                    s = 0;
                else
                    [v, w] = obj.ref.computeControl(t);
                    
                    s = v .* (t - obj.ref.tPause / (obj.ref.ks / obj.ref.kv));
                    ds = s - slast;
                    dt = t - tlast;

                    [vl, vr] = robotModel.VwTovlvr(v, w);

                    th = th + obj.ref.ks * w ./ 2 * dt;
                    x = x + obj.ref.ks * ds * cos(th);
                    y = y + obj.ref.ks * ds * sin(th);
                    th = th + obj.ref.ks * w ./ 2 * dt;
                end
                
                pose = [x; y; th];
                obj.vlSamples(count) = vl;
                obj.vrSamples(count) = vr;
                obj.poseSamples(:, count) = pose;
                obj.tSamples(count) = t;

                % Update for next iter
                count = count + 1;
                tlast = t;
                slast = s;
            end
        end
        
        function [vl, vr] = getVlVrArT(obj, t)
            vl = interp1(obj.tSamples, obj.vlSamples, t);
            vr = interp1(obj.tSamples, obj.vrSamples, t);
        end
            
        function pose = getPoseAtT(obj,t)
            pose = interp1(obj.tSamples, obj.vlSamples, t);
        end
        
    end
end

