classdef estRobot
    % Estimate robot state by doing odometry 
    % from the encoder readings. Doesn't handle the callback
    % so make sure you block on the encoder signals in the main
    % loop
    
    properties
        x
        y
        th
        wheelbase
        initRightEncoder
        initLeftEncoder
        prevLeftEncoder
        prevRightEncoder
        lastTime
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
    end
        
    methods
        function obj = estRobot(wheelbase)
            obj.x = 0;
            obj.y = 0;
            obj.th = 0;
            obj.wheelbase = wheelbase;
            obj.initLeftEncoder = 0;
            obj.initRightEncoder = 0;
            obj.prevLeftEncoder = 0;
            obj.prevRightEncoder = 0;
            obj.lastTime = 0;
        end
        
        function obj = setInitEncoder(obj, lEnc, rEnc, timeStamp)
            obj.initLeftEncoder = lEnc;
            obj.initRightEncoder = rEnc;
            obj.lastTime = timeStamp;
            obj.prevLeftEncoder = 0;
            obj.prevRightEncoder = 0;
        end 
        
        function obj = updatePosition(obj, lEnc, rEnc)
            % pass in the queried odometry at the top of the loop
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
        
        function [x, y, th] = getRobotPose(obj)
            x = obj.x;
            y = obj.y;
            th = obj.th;
        end
    end
end

