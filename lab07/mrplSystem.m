classdef mrplSystem
    % Sub out the main loop when you want to run something
    % Useful coordinate transforms are included
    % Variable coordinate transform convention is Txy, where
    % x is relative to y. For example, Twr is transform of the world
    % relative to the robot
    
    properties(Constant)
        wheelbase = 0.09;
        Vref = 0.2;
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
        Two
        spline
    end
    
    methods
        function obj = mrplSystem(endpoints)
            % endpoints: absolute world coordinates of where 
            % we want to end up, first row always 0, 0, 0; 
            % ex. endpoints = [0,      0,      0;
            %          0.3048, 0.3048, 0.0;
            %         -0.3048, -0.3048, - pi()/2.0;
            %         -0.6096, 0, 0];
            obj.endpoints = endpoints;
            % estBot: odometry/estimated state of the robot
            obj.estRobot = estRobot(obj.wheelbase);
            obj.realX = zeros(1, 1500);
            obj.realY = zeros(1, 1500);
            obj.realTh = zeros(1, 1500);
            obj.refX = zeros(1, 1500);
            obj.refY = zeros(1, 1500);
            obj.refTh = zeros(1, 1500);
            obj.Two = 0;
            obj.spline = 0;
        end
        
        function [V, w] = useFeedback(obj, refPose, acPose)
            % compute the controller with properly adjusted coordinates
            control = controller(refPose, acPose, obj.Vref, tau);
            % produce trajectory follower
            follow = trajectoryFollower(obj.spline, control);
            [V, w] = follow.getRealVw(currT); 
        end
        
        function [xf, yf, thf] = getEndpointToOriginPose(obj, pointNumber)
            i = pointNumber;
            % transform of world wrt robot origin
            origin = obj.endpoints(i-1, :);
            ox = origin(1);
            oy = origin(2);
            oth = origin(3);
            originPose = pose(ox, oy, oth);
            obj.Two = originPose.aToB();

            % transform of endpoint wrt world
            nextPoint = obj.endpoints(i, :);
            nextX = nextPoint(1);
            nextY = nextPoint(2);
            nextTh = nextPoint(3);
            nextPose = pose(nextX, nextY, nextTh);
            Tew = nextPose.bToA();

            % transform of endpoints wrt robot origin
            Toe = obj.Two * Tew;
            endToRobotPose = pose.matToPoseVec(Toe);
            xf = endToRobotPose(1);
            yf = endToRobotPose(2);
            thf = endToRobotPose(3);
        end
        
        function [relx, rely, relth] = getReferenceToWorldPose(obj, currT)
            % coordinates of reference wrt path origin
            robToOriginCoords = obj.spline.getPoseAtTime(currT);
            robToOriginPose = pose(robToOriginCoords(1), robToOriginCoords(2), robToOriginCoords(3));
            Tro = robToOriginPose.bToA;

            % coordinates of reference wrt world
            % finds inverse of ref to origin * origin to world 
            Trw = Tro/obj.Two;
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
            obj.estRobot.updatePosition(left, right);
        end
        
        function plotData(obj)
            clf;
            hold on;
            xlabel('Robot X (meters)');
            ylabel('Robot Y (meters)');
            title('First Section');
            plot(-obj.refY, obj.refX);
            plot(-obj.realY, obj.realX);
            legend({'Reference Path', 'Robot Path'});
        end
        
        function obj = executeTrajectory(obj)
            % lab06 loop
            % have to manually clear all now
            global robot;
            global frames;
            global currLeftEncoder;
            global currRightEncoder;
            global timestamp;
            
            frames = 0;
            timestamp = 0;
            robot = raspbot('RaspBot-16');
            robot.encoders.NewMessageFcn=@encoderEventListener;
            pause(0.5);
            
            initLeftEncoder = currLeftEncoder;
            initRightEncoder = currRightEncoder;
            initTime = timestamp;
            
            % set baseline for state estimator
            obj.estRobot.setInitEncoder(initLeftEncoder, initRightEncoder, initTime);

            xlim([-0.6 0.6]);
            ylim([-0.6 0.6]);
            title('Path of Robot');
            xlabel('x (meters)');
            ylabel('y (meters)');

            positionIdx = 1;
            tau = 85;
            feedBack = false;

            control = 0;
            acPose = 0;
            refPose = 0;
            follow = 0;

            while(frames < 5)
                pause(0.05);
            end
            
            obj.updateStateEst();
            pause(3);
            robot.sendVelocity(0, 0);
            
            for i = 2:4
                sgn = 1;
                [xf, yf, thf] = obj.getEndpointToOriginPose(i);
                
                obj.spline = cubicSpiral.planTrajectory(xf, yf, thf, sgn);
                obj.spline.planVelocities(obj.Vref);

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
                    obj.updateStateEst();
                    
                    % clock time
                    currT = toc(startTic);
                    
                    % get goal position
                    [relx, rely, relth] = obj.getReferenceToWorldPose(currT);
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
                        [V, w] = obj.useFeedback(refPose, acPose);
                    end

                    [vl, vr] = robotModel.VwTovlvr(V, w);
                    positionIdx = positionIdx + 1;

                    robot.sendVelocity(vl, vr);
                    pause(0.05);
                end
                robot.stop();
                pause(0.1);
            end
            robot.stop();
            robot.shutdown();
            
            obj.truncArrays(positionIdx);
            obj.plotData();
        end
        
    end
    
end

           

