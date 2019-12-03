classdef estRobot
    % Estimate robot state by doing odometry 
    % from the encoder readings. Doesn't handle the callback
    % so make sure you block on the encoder signals in the main
    % loop
    
    properties(Constant)
        k = 0.25;
    end
    
    properties
        x
        y
        th
        ids
        wheelbase
        initRightEncoder
        initLeftEncoder
        prevLeftEncoder
        prevRightEncoder
        lastTime
        lineMapLoc
    end
    
    methods(Static)
        function [l, r, time] = getEncData()
            global currRightEncoder
            global currLeftEncoder
            global timestamp
            l = currLeftEncoder;
            r = currRightEncoder;
            time = timestamp;
        end
        
        function [lscan, same] = getLaserData()
            global laserscan
            global samescan
            lscan = laserscan;
            same = samescan;
        end
    end
        
    methods
        function obj = estRobot(wheelbase, lineMapLoc)
            % takes in the params for the initial line map localization
            % as well as the params for the wheelbase size
            obj.x = 0;
            obj.y = 0;
            obj.th = 0;
            obj.wheelbase = wheelbase;
            obj.initLeftEncoder = 0;
            obj.initRightEncoder = 0;
            obj.prevLeftEncoder = 0;
            obj.prevRightEncoder = 0;
            obj.lastTime = 0;
            obj.lineMapLoc = lineMapLoc;
        end
        
        function obj = setInitEncoder(obj, lEnc, rEnc, timeStamp)
            obj.initLeftEncoder = lEnc;
            obj.initRightEncoder = rEnc;
            obj.lastTime = timeStamp;
            obj.prevLeftEncoder = 0;
            obj.prevRightEncoder = 0;
        end 
        
        function obj = updatePositionEnc(obj, lEnc, rEnc)
            % state estimation solely on odometry
            lEnc = lEnc - obj.initLeftEncoder;
            rEnc = rEnc - obj.initRightEncoder;
            lDiff = lEnc - obj.prevLeftEncoder;
            rDiff = rEnc - obj.prevRightEncoder;
            
            thetaChange = (rDiff - lDiff)/obj.wheelbase;
            positionChange = (lDiff + rDiff)/2;
            
            obj.th = obj.th + thetaChange/2;
            obj.x = obj.x + positionChange*cos(obj.th);
            obj.y = obj.y + positionChange*sin(obj.th);
            obj.th = obj.th + thetaChange/2;
            
            obj.prevLeftEncoder = lEnc;
            obj.prevRightEncoder = rEnc;
        end
        
        function obj = updatePositionLidar(obj, rangeIm)
            % state est based solely on laserscan data    
            inPose = pose(obj.x, obj.y, obj.th);
            [succ, outPose, ids] = obj.lineMapLoc.refinePose(inPose, rangeIm, 50);
            if succ
                obj.x = outPose.x();
                obj.y = outPose.y();
                obj.th = outPose.th();
                obj.ids = ids;
            else
                disp("failed to localize");
            end
        end
        
        function obj = updatePositionFusion(obj, rangeIm)
            % assumes the current data for pose is p(est)
            % to be run after updating the position based on odometry
            inPose = pose(obj.x, obj.y, obj.th);
            [succ, outPose, ids] = obj.lineMapLoc.refinePose(inPose, rangeIm, 50);
            if succ
                obj.ids = ids;
                errX = outPose.x() - inPose.x();
                errY = outPose.y() - inPose.y();
                errTh = angleArith(inPose.th(), outPose.th(), -1);
                if abs(errX) > 0.10 || abs(errY) > 0.10
                    [left, right, ~] = estRobot.getEncData();
                    obj = obj.updatePositionEnc(left, right);
                    disp("big diff");
                else
                    obj.x = obj.x - obj.k * errX;
                    obj.y = obj.y - obj.k * errY;
                    obj.th = obj.th - obj.k * errTh;
                end
            else
                [left, right, ~] = estRobot.getEncData();
                obj = obj.updatePositionEnc(left, right); 
                disp("No scan match");
            end
        end
        
        function ids = getFitIds(obj)
            ids = obj.ids;
        end
        
        function obj = setPose(obj, pose)
            obj.x = pose.x();
            obj.y = pose.y();
            obj.th = pose.th();
        end
        
        function [x, y, th] = getRobotPose(obj)
            x = obj.x;
            y = obj.y;
            th = obj.th;
        end
        
        function prw = getRobotPoseObj(obj)
            prw = pose(obj.x, obj.y, obj.th);
        end
    end
end

