classdef mrplSystem
    % Sub out the main loop when you want to run something
    % Useful coordinate transforms are included
    % Variable coordinate transform convention is Txy, where
    % x is relative to y. For example, Twr is transform of the world
    % relative to the robot
    
    properties(Constant)
        wheelbase = 0.09;
        Vref = 0.2;
        % pose of goal to pallet
        Pgp = pose(-0.25, 0, 0);
        % pose of acquisition position to goal
        % pose of the final position wrt to the acquisition position
        Pfp = pose(-0.05, 0, 0);
        % might need to be negative
        Psr = pose(0, 0, -0.08);
        % 0.0337 specifically for Robit 16
        maxLen = 0.14; % test for now
        arrLen = 2000;
        amax = 0.1;
    end
    
    properties
        endpoints
        estRobot
        refX
        refY
        refTh
        realX
        realY
        realTh
        spline
        Tgp
        Tfp
        Tsr
    end
    
    methods
        function obj = mrplSystem(endpoints)
            % endpoints: absolute world coordinates of where 
            % we want to end up, first row always 0, 0, 0; 
            % ex. endpoints = [0,      0,      0;
            %          0.3048, 0.3048, 0.0;
            %         -0.3048, -0.3048, - pi()/2.0;
            %         -0.6096, 0, 0];
            if (nargin == 1)
                obj.endpoints = endpoints;
            else
                obj.endpoints = [0, 0, 0];
            end 
            % estBot: odometry/estimated state of the robot
            obj.estRobot = estRobot(obj.wheelbase);
            % state estimated arrays of data
            obj.realX = zeros(1, obj.arrLen);
            obj.realY = zeros(1, obj.arrLen);
            obj.realTh = zeros(1, obj.arrLen);
            % reference arrays of data
            obj.refX = zeros(1, obj.arrLen);
            obj.refY = zeros(1, obj.arrLen);
            obj.refTh = zeros(1, obj.arrLen);
            obj.spline = 0;
            obj.Tgp = obj.Pgp.bToA;
            obj.Tsr = obj.Psr.bToA;
            obj.Tfp = obj.Pfp.bToA;
        end
        
        function [V, w] = useFeedback(obj, refPose, acPose, tau, currT, ang, lin)
            % compute the controller with properly adjusted coordinates
            control = controller(refPose, acPose, obj.Vref, tau);
            % produce trajectory follower
            follow = trajectoryFollower(obj.spline, control);
            [V, w] = follow.getRealVw(currT, ang, lin); 
        end
        
        function Two = getWorldToOriginT(obj)
            % use for this lab, pass in origin posefor 
            % transform of world wrt robot origin
            [ox, oy, oth] = obj.estRobot.getRobotPose();
            originPose = pose(ox, oy, oth);
            Two = originPose.aToB();
        end
        
        function [xf, yf, thf, Two] = getEndpointToOriginPoint(obj, pointNumber)
            % irrelevant for this lab, as 
            % instead of the absolute world coordinates, simply use the 
            % robot as the origin and plan individual trajectories from
            % passing in the origin pose and final pose
            i = pointNumber;
            % transform of world wrt robot origin
            origin = obj.endpoints(i-1, :);
            ox = origin(1);
            oy = origin(2);
            oth = origin(3);
            originPose = pose(ox, oy, oth);
            Two = originPose.aToB();

            % transform of endpoint wrt world
            nextPoint = obj.endpoints(i, :);
            nextX = nextPoint(1);
            nextY = nextPoint(2);
            nextTh = nextPoint(3);
            nextPose = pose(nextX, nextY, nextTh);
            Tew = nextPose.bToA();

            % transform of endpoints wrt robot origin
            Teo = Two * Tew;
            endToRobotPose = pose.matToPoseVec(Teo);
            xf = endToRobotPose(1);
            yf = endToRobotPose(2);
            thf = endToRobotPose(3);
        end
        
        function [relx, rely, relth] = getReferenceToWorldPose(obj, currT, Two)
            % coordinates of reference wrt path origin
            robToOriginCoords = obj.spline.getPoseAtTime(currT);
            robToOriginPose = pose(robToOriginCoords(1), robToOriginCoords(2), robToOriginCoords(3));
            Tro = robToOriginPose.bToA;

            % coordinates of reference wrt world
            % finds inverse of ref to origin * origin to world 
            Trw = Two\Tro;
            relPose = pose.matToPoseVec(Trw);
            relx = relPose(1);
            rely = relPose(2);
            relth = relPose(3);
        end
        
        function obj = truncArrays(obj, positionIdx)
            obj.refX = obj.refX(1, 1:positionIdx-1);
            obj.refY = obj.refY(1, 1:positionIdx-1);
            obj.refTh = obj.refTh(1, 1:positionIdx-1);
            obj.realX = obj.realX(1, 1:positionIdx-1);
            obj.realY = obj.realY(1, 1:positionIdx-1);
            obj.realTh = obj.realTh(1, 1:positionIdx-1);
        end
        
        function obj = updateStateEstEnc(obj)
            [left, right, ~] = obj.estRobot.getEncData();
            obj.estRobot = obj.estRobot.updatePositionEnc(left, right);
        end
        
        function obj = updateStateEstLidar(obj, lineMapLocalizer, rangeIm)
            rangeHomo = [rangeIm.xArray;
                        
            obj.estRobot = obj.estRobot.updatePositionLidar(lineMapLocalizer, rangeIm);
        end 
        
        function plotData(obj)
            clf;
            hold on;
            xlabel('Robot X (meters)');
            ylabel('Robot Y (meters)');
            title('First Section');
            plot(obj.refX, obj.refY);
            plot(obj.realX, obj.realY);
            legend({'Reference Path', 'Robot Path'});
        end
        
        % sets up finding the position of the sail before driving into it
        function [xf, yf, thf, errf, numf] = findSail(obj, sailFinder)
            roiMidpoints = sailFinder.roiFilter();
            roiMidpoints = roiMidpoints';
            roiSize = size(roiMidpoints, 1);
            errf = 10000;
            xf = 10000;
            yf = 10000;
            thf = 10000;
            numf = 10000;
            for i=1:roiSize
                [x, y, th, err, num] = sailFinder.findLineCandidate(roiMidpoints(i,:), obj.maxLen);
                if err < errf
                    xf = x;
                    yf = y;
                    thf = th;
                    errf = err;
                    numf = num;
                end
            end
            % if nothing was found, loop again
            if xf == 10000
                return
            end
            disp("RANGEIMAGE VALUES");
            disp(xf);
            disp(yf);
            disp(thf);
            % pose of pallet to sensor
            Pps = pose(xf, yf, thf);
            % transform of pallet to sensor
            Tps = Pps.bToA;
            % transform of acquisition position to robot
            Tgr = obj.Tsr * Tps * obj.Tgp;
            Pgr = pose.matToPoseVec(Tgr);
            xf = Pgr(1);
            yf = Pgr(2);
            thf = Pgr(3);
            disp([xf, yf, thf]);
        end 
        
         % sets up finding the position of the sail before driving into it
        function [tx, ty, cx, cy] = justFind(obj, sailFinder)
            roiMidpoints = sailFinder.roiFilter();
            roiMidpoints = roiMidpoints';
            roiSize = size(roiMidpoints, 1);
            errf = 10000;
            xf = 10000;
            yf = 10000;
            thf = 10000;
            numf = 10000;
            for i=1:roiSize
                [x, y, th, err, num] = sailFinder.findLineCandidate(roiMidpoints(i,:), obj.maxLen);
                if err < errf
                    xf = x;
                    yf = y;
                    thf = th;
                    errf = err;
                    numf = num;
                    cx = sailFinder.finalXBox;
                    cy = sailFinder.finalYBox;
                    tx = sailFinder.testXBox;
                    ty = sailFinder.testYBox;
                end
            end
            % if nothing was found, loop again
            if xf == 10000
                return
            end
            disp([xf, yf, thf]);
        end
        
        function [tx, ty, cx, cy, xs, ys] = rangeTesting(obj)
            global robot;
            robot = raspbot('RaspBot-MyBoy');
            robot.startLaser();
            pause(3);
            laserData = robot.laser.LatestMessage.Ranges;
            sailfinder = rangeImage(laserData, 1, true);
            [tx, ty, cx, cy] = obj.justFind(sailfinder);
            xs = sailfinder.xArray;
            ys = sailfinder.yArray;
            pause(1);
            robot.stopLaser();
        end
                
        function obj = runRobot(obj, tau, feedBack, endPoint, ang, lin, sgn)
            global robot;
            global positionIdx;
            Two = obj.getWorldToOriginT();
            xf = endPoint(1);
            yf = endPoint(2);
            thf = endPoint(3);
            
            if (lin)
                obj.spline = trapezoidalReferenceControl(obj.amax, obj.Vref, endPoint(1), sgn, 0);
            elseif (ang)
                obj.spline = rotationalReferenceControl(obj.amax, obj.Vref, endPoint(3), sgn, 0);
            else
                obj.spline = cubicSpiral.planTrajectory(xf, yf, thf, sgn);
                obj.spline.planVelocities(obj.Vref);
            end
            
            % pushes the robot on the actual trajectory
            dur = obj.spline.getTrajectoryDuration();

            currT = 0;
            firstIteration = false;

            while(currT < dur)
                if(firstIteration == false)
                    startTic = tic();        
                    firstIteration = true;
                    continue;
                end

                % update our state estimation
                obj = obj.updateStateEstEnc();

                % clock time
                currT = toc(startTic);

                % get goal position
                [relx, rely, relth] = obj.getReferenceToWorldPose(currT, Two);
                obj.refX(1, positionIdx) = relx;
                obj.refY(1, positionIdx) = rely;
                obj.refTh(1, positionIdx) = relth;

                % get state estimation
                [xReal, yReal, thReal] = obj.estRobot.getRobotPose();
                obj.realX(1, positionIdx) = xReal;
                obj.realY(1, positionIdx) = yReal;
                obj.realTh(1, positionIdx) = thReal;

                % get reference velocities
                V = obj.spline.getVAtTime(currT);
                w = obj.spline.getwAtTime(currT);

                if (feedBack)
                    % compute the controller with properly adjusted coordinates
                    refPose = pose(relx, rely, relth);
                    acPose = pose(xReal, yReal, thReal);
                    [V, w] = obj.useFeedback(refPose, acPose, tau, currT, ang, lin);
                end

                [vl, vr] = robotModel.VwTovlvr(V, w);
                positionIdx = positionIdx + 1;

                if isnan(vl) || isnan(vr)
                    vl = 0;
                    vr = 0;
                end
                robot.sendVelocity(vl, vr);
                pause(0.05);
            end
            robot.stop();
        end
                
        function obj = executeTrajectory(obj)
            % lab06 loop
            % have to manually clear all now
            global robot;
            global frames;
            global currLeftEncoder;
            global currRightEncoder;
            global timestamp;
            global positionIdx;
            
            frames = 0;
            timestamp = 0;
            robot = raspbot('RaspBot-11');
            robot.encoders.NewMessageFcn=@encoderEventListener;
            robot.startLaser();
            
            initLeftEncoder = currLeftEncoder;
            initRightEncoder = currRightEncoder;
            initTime = timestamp;
            
            set baseline for state estimator
            obj.estRobot.initLeftEncoder = initLeftEncoder;
            obj.estRobot.initRightEncoder = initRightEncoder;
            obj.estRobot.lastTime = initTime;
            

            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Path of Robot');
            xlabel('x (meters)');
            ylabel('y (meters)');

            positionIdx = 1;
            tau = 5;
            feedBack = false;

            control = 0;
            acPose = 0;
            refPose = 0;
            follow = 0;

            while(frames < 5)
                pause(0.05);
            end
            
            obj = obj.updateStateEstEnc();
            pause(5);
            robot.sendVelocity(0, 0);
            i = 0;
            while i < 3
                laserData = robot.laser.LatestMessage.Ranges;
                sailFinder = rangeImage(laserData, 1, true);
                % returns Pgr, pose of goal to robot (not the actual
                % pallet)
                [xf, yf, thf] = obj.findSail(sailFinder)
                % if nothing was found, redo it
                if xf == 10000
                    continue
                end
                % plan and run the trajectory to the acquisition pose
                obj = obj.runRobot(tau, true, [xf, yf, thf], false, false, 1);
                pause(1);
                
                % go on straight line trajectory
                % position of goal to robot in acquisition pose, so ideally
                % straight on

%                 xf = 0.05;
%                 if (thf > 0.1 || thf < -0.1)
%                     if (thf > 0)
%                         obj = obj.runRobot(tau, feedBack, [0, 0, abs(thf)], true, false, -1);
%                     else
%                         obj = obj.runRobot(tau, feedBack, [0, 0, abs(thf)], true, false, 1);
%                     end
%                 end    
%                 
                % assume straight line, if not then add in the small angle
%                 [xf, yf, thf] = obj.findSail(sailFinder);
                % movements
                % Make this a damn function
                laserData = robot.laser.LatestMessage.Ranges;
                sailFinder = rangeImage(laserData, 1, true);
                [xf, yf, thf] = obj.findSail(sailFinder);
                Pgr = pose(xf, yf, thf);
                % transform of pallet to robot 
                Tpr = Pgr.bToA / obj.Tgp;
                % transform of ideal final (distance + 5 cm) to robot
                Tfr = Tpr * obj.Tfp;
%                 Pgr = pose(xf, yf, thf);
%                 Tpr = Pgr.bToA;
                Pfr = pose.matToPoseVec(Tfr);
                xf = Pfr(1);
                yf = Pfr(2);
                thf = Pfr(3);
                obj = obj.runRobot(tau, feedBack, [xf, yf, thf], false, false, 1);
                pause(1);
                robot.forksUp();
                pause(1);
                robot.forksDown();
                pause(1);
                % back up 5 cm
                xf = 0.05;
                obj = obj.runRobot(tau, feedBack, [xf, 0, 0], false, true, -1);
                % spin around
                thf = pi;
                obj = obj.runRobot(tau, feedBack, [0, 0, thf], true, false, 1);
                % robot.forksDown();
                pause(10);
                i = i + 1;
            end
            robot.stopLaser();
            robot.stop();
            robot.shutdown();
            xErr = obj.refX(positionIdx - 1) - obj.realX(positionIdx - 1);
            yErr = obj.refY(positionIdx - 1) - obj.realY(positionIdx - 1);
            % display(sqrt(xErr*xErr + yErr*yErr));
            obj = obj.truncArrays(positionIdx);
            obj.plotData();
        end
        
        function filterData = filterLaserData(obj, laserData)
            filterData = laserData(1:10:size(laserData, 2));
        end
            
        
        function obj = teleopAndMapLidarOnly(obj)
            global robot;
            robot = raspbot('RaspBot-11');
            robot.startLaser();
            pause(3);
            
            xlim([-0.1 1.2]);
            ylim([-0.1 1.2]);
            title('Path of Robot');
            xlabel('x (meters)');
            ylabel('y (meters)');
            
            % figure out how to actually get these lines in, l1 l2 being
            % the maps
            l1 = [0.05, 0; 
                  0, 0.05];
            l2 = [1.219, 0;
                  0, 1.219];
            % points of the walls wrt to the world
            gain = 0.3;
            errThresh = 0.01;
            gradThresh = 0.0005;
            vGain = 1.25;
            
            localizer = lineMapLocalizer(l1, l2, gain, errThresh, gradThresh);
            keyDriver = robotKeypressDriver(gcf);
            pause(2);
            
            figure(1);
            hold on;
            plot([l1(1, 1), l2(1, 1)], [l1(2, 1), l2(2, 1)], 'b');
            plot([l1(1, 2), l2(1, 2)], [l1(2, 2), l2(2, 2)], 'b');
            initPose = pose(15*0.0254,9*0.0254,pi()/2.0);
            obj.estRobot = obj.estRobot.setPose(initPose);
            
            % run once to make the thing
            laserPointsWorldFrame = [];
            lxyHandle = [];
            
            robotPointsWorldFrame = [];
            rxyHandle = [];
            
            while true
                if size(laserPointsWorldFrame, 2) > 0
                    delete(lxyHandle);
                end    
                if size(robotPointsWorldFrame, 2) > 0
                    delete(rxyHandle);
                end
                
                robotKeypressDriver.drive(robot, vGain);
                
                laserData = robot.laser.LatestMessage.Ranges;
                % get the points in the rangeImage
                rangeIm = rangeImage(laserData, 10, true);
                % set up the 
                laserPointsRobotFrame = [rangeIm.xArray;
                                         rangeIm.yArray;
                                         ones(1, size(rangeIm.xArray, 2))];
                obj = obj.updateStateEstLidar(localizer, laserPointsRobotFrame);
                % x, y, th of robot wrt world from the localization
                [xrw, yrw, thrw] = obj.estRobot.getRobotPose();
                idxs = obj.estRobot.getFitIds();
                % pose of robot wrt world
                Prw = pose(xrw, yrw, thrw);
                Trw = Prw.bToA();
                
                bodyPts = robotModel.bodyGraph();
                robotPointsWorldFrame = Trw * bodyPts;
                laserPointsWorldFrame = Trw * laserPointsRobotFrame(:, idxs);
                
                xRobot = robotPointsWorldFrame(1, :);
                yRobot = robotPointsWorldFrame(2, :);
                rxyHandle = plot(xRobot, yRobot, 'g');
                
                xWorld = laserPointsWorldFrame(1, :);
                yWorld = laserPointsWorldFrame(2, :);
                lxyHandle = scatter(xWorld, yWorld, 'r');
                pause(0.05);
            end
        end
    end
    
end

           

