clear all;
clf;
global prevTimestamp;
global prevLeftEncoder;
global prevRightEncoder;
global robot;
global initLeftEncoder;
global initRightEncoder;
global frames

wheelbase = 0.09;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;

robot = raspbot('RaspBot-16');
initLeftEncoder = robot.encoders.LatestMessage.Vector.X;
initRightEncoder = robot.encoders.LatestMessage.Vector.Y;
robot.encoders.NewMessageFcn=@encoderEventListener;

pause(2)

xlim([-0.6 0.6]);
ylim([-0.6 0.6]);
title('Path of Robot');
xlabel('x (meters)');
ylabel('y (meters)');

params = [0,      0,      0;
          0.3048, 0.3048, 0.0;
         -0.6096, -0.6096, - pi()/2.0;
         -0.3048, 0.3048, pi()/2.0];

Vref = 0.2;
     
positionIdx = 1;
currT = 0;
prevT = 0;
firstIteration = false;

refXArr = zeros(1, 1000);
refYArr = zeros(1, 1000);
refThArr = zeros(1, 1000);
realX = zeros(1, 1000);
realY = zeros(1, 1000);
realTh = zeros(1, 1000);

tau = 85;
feedBack = false;

pause(3);

lastT = 0;
thReal = 0;
yReal = 0;
xReal = 0;

control = 0;
acPose = 0;
refPose = 0;
follow = 0;

while(frames < 5)
    pause(0.05);
end

robot.sendVelocity(0, 0);

for i = 2:4
    sgn = 1;
    executeParam = params(i, :)';
    
    xf = executeParam(1);
    yf = executeParam(2);
    thf = executeParam(3);
    
    spline = cubicSpiral.planTrajectory(xf, yf, thf, sgn);
    spline.planVelocities(Vref);
    
    dur = spline.getTrajectoryDuration();
    
    currT = 0;
    firstIteration = false;
    lastT = 0;
    
    % coordinates of path origin wrt world
    origin = sum(params(1:i-1, :), 1);
    ox = origin(1);
    oy = origin(2);
    oth = origin(3);
    originPose = pose(ox, oy, oth);
    H = originPose.bToA();
    
    while(currT < dur)
        if(firstIteration == false)
            startTic = tic();        
            firstIteration = true;
            continue;
        end
        
        currT = toc(startTic);
        
        % coordinates of robot wrt world
        robCoords = spline.getPoseAtTime(currT);
        robPose = pose(robCoords(1), robCoords(2), robCoords(3));
        robMat = robPose.bToA;
        
        % coordinates of robot wrt path origin
        relMat = H * robMat;
        relPose = pose.matToPoseVec(relMat);
        relx = relPose(1);
        rely = relPose(2);
        relth = relPose(3);
        
        xReal = x;
        yReal = y;
        thReal = theta;
        
        realX(1, positionIdx, i-1) = xReal;
        realY(1, positionIdx, i-1) = yReal;
        realTh(1, positionIdx, i-1) = thReal;

        refXArr(1, positionIdx, i-1) = relx;
        refYArr(1, positionIdx, i-1) = rely;
        refThArr(1, positionIdx, i-1) = relth;
        
        V = spline.getVAtTime(currT);
        w = spline.getwAtTime(currT);
        
        if (feedBack)
            % compute the controller with properly adjusted coordinates
            refPose = pose(relx, rely, relth);
            acPose = pose(xReal, yReal, thReal);
            control = controller(refPose, acPose, Vref, tau);
            
            % produce trajectory follower
            follow = trajectoryFollower(spline, control);
            [V, w] = follow.getRealVw(currT); 
        end
        
        myVarr(1, positionIdx) = V;
        mywarr(1, positionIdx) = w;
        
        [vl, vr] = robotModel.VwTovlvr(V, w);
        myvla(1, positionIdx) = vl;
        myvra(1, positionIdx) = vr;
        
        positionIdx = positionIdx + 1;

        robot.sendVelocity(vl, vr);
        pause(0.05);
    end
    robot.stop();
    pause(0.1);
end

robot.stop();
robot.shutdown();

refXArr = refXArr(:, 1:positionIdx-1, :);
refYArr = refYArr(:, 1:positionIdx-1, :);
refThArr = refThArr(:, 1:positionIdx-1, :);
realX = realX(:, 1:positionIdx-1, :);
realY = realY(:, 1:positionIdx-1, :);
realTh = realTh(:, 1:positionIdx-1, :);

% tarr = tarr(1:positionIdx-1);
% vla = vla(1:positionIdx-1);
% vra = vra(1:positionIdx-1);

figure(1)
clf;
hold on;
xlabel('Robot X (meters)');
ylabel('Robot Y (meters)');
title('First Section');
indX = refXArr(1, :, 1) ~= 0;
indY = refYArr(1, :, 1) ~= 0;
refX1 = refXArr(:, indX, 1);
refY1 = refYArr(:, indY, 1);
realX1 = realX(:, indX, 1);
realY1 = realY(:, indY, 1);
plot(refX1, refY1);
plot(realX1, realY1);
legend({'Reference Path', 'Robot Path'});

figure(2)
clf;
hold on;
xlabel('Robot X (m)');
ylabel('Robot Y (m)');
title('Second Section');
indX = refXArr(1, :, 2) ~= 0;
indY = refYArr(1, :, 2) ~= 0;
refX2 = refXArr(:, indX, 2);
refY2 = refYArr(:, indY, 2);
realX2 = realX(:, indX, 2);
realY2 = realY(:, indY, 2);
plot(refX2, refY2);
plot(realX2, realY2);
legend({'Reference Path', 'Robot Path'});

figure(3)
clf;
hold on;
hold on;
xlabel('Robot X (m)');
ylabel('Robot Y (m)');
indX = refXArr(1, :, 3) ~= 0;
indY = refYArr(1, :, 3) ~= 0;
refX3 = refXArr(:, indX, 3);
refY3 = refYArr(:, indY, 3);
realX3 = realX(:, indX, 3);
realY3 = realY(:, indY, 3);
plot(refX3, refY3);
plot(realX3, realY3);
legend({'Reference Path', 'Robot Path'});