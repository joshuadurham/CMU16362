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
        Pgp = pose(-0.125, 0, 0);
        % pose of acquisition position to goal
        % pose of the final position wrt to the acquisition position
        Pfp = pose(-0.1, 0, 0);
        % might need to be negativefv
        Psr = pose(0, 0, -0.08);
        % 0.0337 specifically for Robit 16
        maxLen = 0.125; % test for now
        arrLen = 2000;
        amax = 0.1;
    end
    
    properties
        endpoints
        palletPoints
        dropPoints
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
    
    methods(Static)
        % NOTE: This has 12 foot long side walls, as 
        % we will have for the competition, if things
        % are jank bc you're running on a half field,
        % replace the 3.6576 with 1.8288 for 6 foot wallsf
        function lineMapLoc = makeLocalizer()
            l1 = [0.05, 0; 
                  0, 0.05;
                  2.438, 0.05]';
            l2 = [2.388, 0;
                  0, 3.6576;
                  2.438, 3.6576]';
            % points of the walls wrt to the world
            gain = 0.3;
            errThresh = 0.015;
            gradThresh = 0.0005;
            lineMapLoc = lineMapLocalizer(l1, l2, gain, errThresh, gradThresh);
        end
    end
    
    methods
        function obj = mrplSystem(palletPoints, dropPoints)
            % endpoints: absolute world coordinates of where 
            % we want to end up, first row always 0, 0, 0; 
            % ex. endpoints = [0,      0,      0;
            %          0.3048, 0.3048, 0.0;
            %         -0.3048, -0.3048, - pi()/2.0;
            %         -0.6096, 0, 0];
            if (nargin == 2)
                obj.palletPoints = palletPoints;
                obj.dropPoints = dropPoints;
            else
                obj.palletPoints = [0, 0, 0];
                obj.dropPoints = [0, 0, 0];
            end 
            % estBot: odometry/estimated state of the robot
            obj.estRobot = estRobot(obj.wheelbase, mrplSystem.makeLocalizer());
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
            % transform of world wrt robot origin
            [ox, oy, oth] = obj.estRobot.getRobotPose();
            originPose = pose(ox, oy, oth);
            Two = originPose.aToB();
        end
        
        function [xf, yf, thf, Two] = getEndpointToRobotOriginPoint(obj, pointNumber)
            % plan trajectories from current position to absolute world coordinates
            % by transforming to pose of endpoint in robot frame
            % passing in the origin pose and final pose
            % transform of world wrt robot origin
            originPose = obj.estRobot.getRobotPoseObj();
            Two = originPose.aToB();

            % transform of endpoint wrt world
            nextPoint = obj.endpoints(pointNumber, :);
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
        
        
        function [xf, yf, thf, Two] = getEndpointToRefOriginPoint(obj, pointNumber)
            % absolute world coordinates, simply use the 
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
        
        function obj = updateStateEstLidar(obj)
            global robot;
            lscan = robot.laser.LatestMessage.Ranges;
            rangeIm = rangeImage(lscan, 10, true, false);
            filteredPoints = [rangeIm.xArray;
                              rangeIm.yArray;
                              ones(1, size(rangeIm.xArray, 2))];
            obj.estRobot = obj.estRobot.updatePositionLidar(filteredPoints);
        end
        
        function obj = updateStateEstFusion(obj)
            [left, right, ~] = obj.estRobot.getEncData();
            obj.estRobot = obj.estRobot.updatePositionEnc(left, right);
            [lscan, ~] = obj.estRobot.getLaserData(); %robot.laser.LatestMessage.Ranges;
            rangeIm = rangeImage(lscan, 10, true, false);
            filteredPoints = [rangeIm.xArray;
                              rangeIm.yArray;
                              ones(1, size(rangeIm.xArray, 2))];
            obj.estRobot = obj.estRobot.updatePositionFusion(filteredPoints);
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
        
        % checks to see if you are currently carrying a pallet
        % if doesn't work, run a line detection instead
        function carrying = carryingPallet(obj)
%             [lscan, ~] = obj.estRobot.getLaserData();
%             sailFinder = rangeImage(lscan, 1, false, true);
%             sailFinder.xArray
%             if size(sailFinder.rArray, 2) > 5
%                     carrying = true;
%             % if nothing was found, loop again
%             else
%                 carrying = false;
%             end
            % dirty fix
            carrying = true;
        end 
        
        % simple test to see if carrying a pallet or not
        function obj = testCarrying(obj)
            global robot
            global laserscan
            robot = raspbot('RaspBot-MyBoy');
            robot.startLaser();
            laserscan = zeros(1, 360);
            robot.laser.NewMessageFcn=@laserEventListener;
            pause(5);
            carry = obj.carryingPallet();
            disp(carry);
            robot.stopLaser();
            robot.shutdown();
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
            sailfinder = rangeImage(laserData, 1, true, false);
            [tx, ty, cx, cy] = obj.justFind(sailfinder);
            xs = sailfinder.xArray;
            ys = sailfinder.yArray;
            pause(1);
            robot.stopLaser();
        end
        
        function obj = smallMotions(obj, tau, smallFeedBack)
            global robot;
            robot.forksUp();
            pause(1);
            robot.forksDown();
            pause(1);
            % back up 5 cm
            xf = 0.05;
            obj = obj.runRobot(tau, false, [xf, 0, 0], false, true, -1);
            % spin around
            thf = pi;
            obj = obj.runRobot(tau, false, [0, 0, thf], true, false, 1);
        end
        
        function [x, y, th] = getSailPosition(obj)                
            [lscan, ~] = obj.estRobot.getLaserData();
            sailFinder = rangeImage(lscan, 1, true, false);
            [x, y, th] = obj.findSail(sailFinder);
        end
                
        function obj = runRobot(obj, tau, feedBack, endPoint, ang, lin, sgn)
            global robot;
            global positionIdx;
            Two = obj.getWorldToOriginT();
            xf = endPoint(1);
            yf = endPoint(2);
            thf = endPoint(3);
            
            if (lin && sgn == -1)
                load('smallMotions.mat', 'backup');
                obj.spline = backup;
            elseif (lin && sgn == 1)
                load('smallMotions.mat', 'forward');
                obj.spline = forward;
            elseif (ang)
                load('smallMotions.mat', 'turnAround');
                obj.spline = turnAround;
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

                % update our state estimation; if no feedback (small
                % motion) only update with odometry, otherwise use fusion
%                 if ~feedBack
                obj = obj.updateStateEstEnc();
%                 else
%                     obj = obj.updateStateEstFusion();
%                 end

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
            % lab11 loop
            % have to manually clear all now
            global robot;
            global frames;
            global currLeftEncoder;
            global currRightEncoder;
            global timestamp;
            global positionIdx;
            global laserscan;
            global samescan;
            
            frames = 0;
            timestamp = 0;
            robot = raspbot('RaspBot-11');
            laserscan = zeros(1, 360);
            samescan = true;
            disp("laser");
            robot.startLaser();
            robot.encoders.NewMessageFcn=@encoderEventListener;
            %robot.laser.NewMessageFcn=@laserEventListener;
            
            % get the current ENC values
            initLeftEncoder = currLeftEncoder;
            initRightEncoder = currRightEncoder;
            initTime = timestamp;
            
            %set baseline for state estimator
            obj.estRobot.initLeftEncoder = initLeftEncoder;
            obj.estRobot.initRightEncoder = initRightEncoder;
            obj.estRobot.lastTime = initTime;
            
            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Path of Robot');
            xlabel('x (meters)');
            ylabel('y (meters)');

            positionIdx = 1;
            tau = 4;
            largeMotionFeedBack = true;
            smallMotionFeedBack = false;

            control = 0;
            acPose = 0;
            refPose = 0;
            follow = 0;

            while(frames < 5)
                pause(0.05);
            end
            
            initPose = pose(0.2286,0.2286,3.0*pi()/2.0);
            obj.estRobot = obj.estRobot.setPose(initPose);
            pause(5);            
            robot.sendVelocity(0, 0);
            disp("check");
            palletIdx = 1;
            dropIdx = 1;
            while palletIdx <= size(obj.palletPoints,1) && dropIdx < size(obj.dropPoints, 1)
                i = 2;
                % endPoints(2:end,:) encodes the series of moves for picking up and
                % dropping off a pallet, endPoints(1,:) encodes a safe
                % middle position we can go to if we fail to pick up a
                % pallet, and just try again from there.  It's jank but it
                % should work (hopefully)
                obj.endpoints = [
                    48*0.0254, 36*0.0254, 3*pi()/2.0;
                    1,      1,      1;
                    obj.palletPoints(palletIdx,1), obj.palletPoints(palletIdx,2), obj.palletPoints(palletIdx,3);
                    3,      3,      3;
                    -1,     -1,     -1;
                    obj.dropPoints(dropIdx,1), obj.dropPoints(dropIdx,2), obj.dropPoints(dropIdx,3);
                    2,      2,      2;
                ];
                while i <= size(obj.endpoints, 1)
                    if obj.endpoints(i,1) == -1 && obj.endpoints(i,2) == -1 && obj.endpoints(i,3) == -1
                        % -1, -1, -1 encodes spin to 3pi/2
                        % I assume that we want to keep things at the same
                        % (x,y) coord
                        thf = pi();
                        [ox, oy, ~] = obj.estRobot.getRobotPose();
                        obj = obj.runRobot(tau, smallMotionFeedBack, [0, 0, thf], true, false, 1);
                        %% TODO @BRANDON: look at the values we get from the laser scanner when carrying a pallet,
                        %% and have carryingPallet() return true if we have a pallet, and false otherwise
                        if obj.carryingPallet()
                            % we got a pallet, yay!  Let's go to the next
                            % dropoff point and score that pallet
                            dropIdx = dropIdx + 1;
                        else
                            % we didn't get a pallet sad :(, let's go to
                            % the safe location and try again for the next
                            % pallet
                            obj = obj.updateStateEstEnc();
                            [xf, yf, thf] = obj.getEndpointToRobotOriginPoint(1);
                            obj = obj.runRobot(tau, largeMotionFeedBack, [xf, yf, thf], false ,false, 1);
                            % set i to be big so the loop terminates
                            i = size(obj.endpoints, 1) + 1;
                        end
                    elseif obj.endpoints(i,1) == 1 && obj.endpoints(i,2) == 1 && obj.endpoints(i,3) == 1
                        % 1, 1, 1 encodes spin to pi/2
                        % I assume that we want to keep things at the same
                        % (x,y) coord
                        thf = pi();
                        [ox, oy, ~] = obj.estRobot.getRobotPose();
                        obj = obj.runRobot(tau, smallMotionFeedBack, [0, 0, thf], true, false, 1);
                    elseif obj.endpoints(i,1) == 2 && obj.endpoints(i,2) == 2 && obj.endpoints(i,3) == 2
                        % 2, 2, 2 encodes drop off pallet
                        robot.forksDown();
                        pause(1);
                        % back up 5 cm by driving upward
                        xf = 0.05;
                        [ox, oy, oth] = obj.estRobot.getRobotPose();
                        obj = obj.runRobot(tau, smallMotionFeedBack, [xf, 0, 0], false, true, -1);
                    elseif obj.endpoints(i,1) == 3 && obj.endpoints(i,2) == 3 && obj.endpoints(i,3) == 3
                        % 3, 3, 3 encodes use laser to pick up pallet
                        %% OPTIONAL TODO @BRANDON: we might have to add a second version
                        %% of findSail for the 3 pallets on the wall if we keep 
                        %% detecting the wall and trying to pick up the wall on accident.
                        %% Setting the tuning parameters back to their values for the old lab
                        %% and adding a boolean flag for which set of parameters to use
                        %% should fix that issue, if we really need to (hopefully we don't)
                        robot.forksDown();
                        laserData = robot.laser.LatestMessage.Ranges;
                        % display(laserData);
                        sailFinder = rangeImage(laserData, 1, true, false);
                        [xf, yf, thf] = obj.findSail(sailFinder);
                        display(xf);
                        display(yf);
                        display(thf);
                        if (xf == 10000)
                            break;
                        end
                        Pgr = pose(xf, yf, thf);
                        % transform of pallet to robot 
                        Tpr = Pgr.bToA / obj.Tgp;
                        % transform of ideal final (distance + 5 cm) to robot
                        Tfr = Tpr * obj.Tfp;
        %               Pgr = pose(xf, yf, thf);
        %               Tpr = Pgr.bToA;
                        Pfr = pose.matToPoseVec(Tfr);
                        xf = Pfr(1);
                        yf = Pfr(2);
                        thf = Pfr(3);
                        obj = obj.runRobot(tau, largeMotionFeedBack, [xf, yf, thf], false, false, 1);
                        pause(0.5);
                        xf = 0.04;
                        obj = obj.runRobot(tau, smallMotionFeedBack, [xf, 0, 0], false, true, 1);
                        pause(0.5);
                        robot.forksUp();
                        pause(1);
                    else
                        obj = obj.updateStateEstEnc();
                        [xf, yf, thf] = obj.getEndpointToRobotOriginPoint(i);
                        obj = obj.runRobot(tau, largeMotionFeedBack, [xf, yf, thf], false ,false, 1);
                    end
                    i = i + 1;
                    pause(1);
                end
                palletIdx = palletIdx + 1;
            end
            robot.stopLaser();
            robot.stop();
            robot.shutdown();
            xErr = obj.refX(positionIdx - 1) - obj.realX(positionIdx - 1);
            yErr = obj.refY(positionIdx - 1) - obj.realY(positionIdx - 1);
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
                obj = obj.updateStateEstLidar(laserPointsRobotFrame);
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

           

