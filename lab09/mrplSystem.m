classdef mrplSystem
    % Sub out the main loop when you want to run something
    % Useful coordinate transforms are included
    % Variable coordinate transform convention is Txy, where
    % x is relative to y. For example, Twr is transform of the world
    % relative to the robot
    
    properties(Constant)
        wheelbase = 0.09;
        Vref = 0.2;
        Pgp = pose(-0.075, 0, 0);
        % might need to be negative
        Psr = pose(0, 0, -0.0350);
        % 0.0337 specifically for Robit 16
        maxLen = 0.13; % test for now
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
        
        function obj = updateStateEst(obj)
            [left, right, ~] = estRobot.getEncData();
            obj.estRobot = obj.estRobot.updatePosition(left, right);
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
            % pose of pallet to sensor
            Pps = pose(xf, yf, thf);
            % transform of pallet to sensor
            Tps = Pps.bToA;
            % transform of goal to robot
            Tgr = obj.Tsr * Tps * obj.Tgp;
            Pgr = pose.matToPoseVec(Tgr);
            xf = Pgr(1);
            yf = Pgr(2);
            thf = Pgr(3);
            disp([xf, yf, thf]);
        end 
        
        function obj = runRobot(obj, tau, feedBack, endPoint, ang, lin, sgn)
            global robot;
            global positionIdx;
            Two = obj.getWorldToOriginT();
            xf = endPoint(1);
            yf = endPoint(2);
            thf = endPoint(3);
            sgn = 1;
            
            if (lin)
                obj.spline = trajectoryReferenceControl(obj.amax, obj.Vref, endPoint(1), sgn, 0);
            elseif (ang)
                
            else
                obj.spline = cubicSpiral.planTrajectory(xf, yf, thf, sgn);
                obj.spline.planVelocities(obj.Vref);
            
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
                obj = obj.updateStateEst();

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
                    [V, w] = obj.useFeedback(refPose, acPose, tau, currT);
                end

                [vl, vr] = robotModel.VwTovlvr(V, w);
                positionIdx = positionIdx + 1;

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
            robot = raspbot('RaspBot-16');
            robot.encoders.NewMessageFcn=@encoderEventListener;
            robot.startLaser();
            
            initLeftEncoder = currLeftEncoder;
            initRightEncoder = currRightEncoder;
            initTime = timestamp;
            
            % set baseline for state estimator
            obj.estRobot.initLeftEncoder = initLeftEncoder;
            obj.estRobot.initRightEncoder = initRightEncoder;
            obj.estRobot.lastTime = initTime;
            

            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Path of Robot');
            xlabel('x (meters)');
            ylabel('y (meters)');

            positionIdx = 1;
            tau = 4.1281;
            feedBack = true;

            control = 0;
            acPose = 0;
            refPose = 0;
            follow = 0;

            while(frames < 5)
                pause(0.05);
            end
            
            obj = obj.updateStateEst();
            pause(5);
            robot.sendVelocity(0, 0);
            i = 0;
            while i < 3
                laserData = robot.laser.LatestMessage.Ranges;
                sailFinder = rangeImage(laserData, 1, true);
                % returns Pgr, pose of goal to robot (not the actual
                % pallet)
                [xf, yf, thf] = obj.findSail(sailFinder);
                % if nothing was found, redo it
                if xf == 10000
                    continue
                end
                % plan and run the trajectory to the goal 
                obj = obj.runRobot(tau, feedBack, [xf, yf, thf]);
                pause(1);
                % now, get to pallet position and pick up
                % obj = obj.runRobot(tau, feedBack, [0.02, 0, 0]);
                % robot.forksUp();
                pause(2);
                % robot.forksDown();
                if (i ~= 3)
                    pause(15);
                end
                i = i + 1;
                
% The commented section should be handled by calling obj.runRobot(Two,
% tau, endPose, positionIdx). Once confirmed to work, delete this section of code            

%                 dur = obj.spline.getTrajectoryDuration();
% 
%                 currT = 0;
%                 firstIteration = false;
                
                

%                 while(currT < dur)
%                     if(firstIteration == false)
%                         startTic = tic();        
%                         firstIteration = true;
%                         continue;
%                     end
%                     
%                     % update our state estimation
%                     obj = obj.updateStateEst();
%                     
%                     % clock time
%                     currT = toc(startTic);
%                     
%                     % get goal position
%                     [relx, rely, relth] = obj.getReferenceToWorldPose(currT, Two);
%                     obj.refX(1, positionIdx) = relx;
%                     obj.refY(1, positionIdx) = rely;
%                     obj.refTh(1, positionIdx) = relth;
%                     
%                     % get state estimation
%                     [xReal, yReal, thReal] = obj.estRobot.getRobotPose();
%                     obj.realX(1, positionIdx) = xReal;
%                     obj.realY(1, positionIdx) = yReal;
%                     obj.realTh(1, positionIdx) = thReal;
% 
%                     % get reference velocities
%                     V = obj.spline.getVAtTime(currT);
%                     w = obj.spline.getwAtTime(currT);
% 
%                     if (feedBack)
%                         % compute the controller with properly adjusted coordinates
%                         refPose = pose(relx, rely, relth);
%                         acPose = pose(xReal, yReal, thReal);
%                         [V, w] = obj.useFeedback(refPose, acPose, tau, currT);
%                     end
% 
%                     [vl, vr] = robotModel.VwTovlvr(V, w);
%                     positionIdx = positionIdx + 1;
% 
%                     robot.sendVelocity(vl, vr);
%                     pause(0.05);
%                 end
%                 robot.stop();
                
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
        
    end
    
end

           

