classdef figure8
    
    properties
        % Constants
        vel = 0.2;
        sf = 1;
        length = 10000;
        Tdelay = 0;
        
        % TBD variables
        tf
        kth
        kk
        xArr
        yArr
        thArr
        tArr
        vlArr
        vrArr
        ks
        kv
        tPause
        Tf
    end
        
    methods
        function obj = figure8(Ks, Kv, tPause)
            obj.tf = obj.sf/obj.vel;
            obj.kth = 2 * pi ./ obj.sf;
            obj.kk = 15.1084;
            obj.ks = Ks;
            obj.kv = Kv;
            obj.tPause = tPause;
            % init data arrays
            obj.xArr = zeros(obj.length);
            obj.yArr = zeros(obj.length);
            obj.thArr = zeros(obj.length);
            obj.vlArr = zeros(obj.length);
            obj.vrArr = zeros(obj.length);
            obj.tArr = zeros(obj.length);
            obj.Tf = (Ks / Kv) * obj.tf;

            count = 1;
            x = 0;
            y = 0;
            th = 0;

            tlast = 0;
            slast = 0;
            T = 0;
            first = true;

            while(T < obj.Tf + 2 * obj.tPause)
                if(first)
                    startTic = tic();        
                    first = false;
                end
                T = toc(startTic) - obj.Tdelay;
                t = T / (obj.ks / obj.kv);
                dt = t - tlast;
                if ((T <= obj.tPause) || (T >= obj.Tf + obj.tPause))
                    vl = 0;
                    vr = 0;
                    s = 0;
                else
%                   v = obj.vel;
                    [v, w] = obj.computeControl(t);
                    s = v .* (t - obj.tPause / (obj.ks / obj.kv));
                    ds = s - slast;
%                     k = (obj.kk ./ obj.ks) * sin(obj.kth * s);
%                     w = k .* v;
                    

                    [vl, vr] = robotModel.VwTovlvr(v, w);

                    th = th + obj.ks * w ./ 2 * dt;
                    x = x + obj.ks * ds * cos(th);
                    y = y + obj.ks * ds * sin(th);
                    th = th + obj.ks * w ./ 2 * dt;
                end
                
                % update arrays
                obj.vlArr(count) = vl;
                obj.vrArr(count) = vr;
                obj.xArr(count) = x;
                obj.yArr(count) = y;
                obj.thArr(count) = th;
                obj.tArr(count) = t;

                % Update for next iter
                count = count + 1;
                tlast = t;
                slast = s;

                pause(0.05);
            end
            
            % Contract arrays
            obj.vlArr = obj.vlArr(1:count-1);
            obj.vrArr = obj.vrArr(1:count-1);
            obj.xArr = obj.xArr(1:count-1);
            obj.yArr = obj.yArr(1:count-1);
            obj.thArr = obj.thArr(1:count-1);
            obj.tArr = obj.tArr(1:count-1);
        end
        
        function [V, w] = computeControl(obj, timeNow)
%             if ((timeNow <= obj.tPause) || (timeNow >= obj.Tf + obj.tPause))
%                 V = 0;
%                 w = 0;
%                 return
%             end
            V = obj.vel;
            t = timeNow - (obj.Tdelay / (obj.ks / obj.kv));
            s = V .* (t - obj.tPause / (obj.ks / obj.kv));
            k = (obj.kk ./ obj.ks) * sin(obj.kth * s);
            V = V * obj.kv;
            w = k .* V;
%             [~, ind] = min(abs(obj.tArr - timeNow));
%             velL = obj.vlArr(ind);
%             velR = obj.vrArr(ind);
        end
        
        function duration = getTrajectoryDuration(obj)
            duration = obj.tPause * 2 + obj.Tf;
        end
    end
end
    

            
            
            

