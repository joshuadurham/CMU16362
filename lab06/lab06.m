clear all;
clf;
global wheelbase;
global prevTimestamp;
global prevLeftEncoder;
global prevRightEncoder;
global x;
global y;
global theta;
global robot;
global initLeftEncoder;
global initRightEncoder;
global frames

wheelbase = 0.09;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;
x = 0;
y = 0;
theta = 0;
frames = 0;

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

% vla = zeros(1, 1000);
% vra = zeros(1, 1000);
% tarr = zeros(1, 1000);

refXArr = zeros(1, 1000, 3);
refYArr = zeros(1, 1000, 3);
refThArr = zeros(1, 1000, 3);
realX = zeros(1, 1000, 3);
realY = zeros(1, 1000, 3);
realTh = zeros(1, 1000, 3);

tau = 85;
feedBack = true;

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
    
    while(currT < dur)
        if(firstIteration == false)
            startTic = tic();        
            firstIteration = true;
            continue;
        end
        currT = toc(startTic);
        dt = currT - lastT;
        
        % coordinates of path origin wrt world
        origin = params(i-1, :)';
        ox = origin(1);
        oy = origin(2);
        oth = origin(3);
        originPose = pose(ox, oy, oth);
        
        % coordinates of robot wrt world
        robPose = spline.getPoseAtTime(time);
        
        % coordinates of robot wrt path origin
        H = originPose.aToB();
        relPose = H * robPose;
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
            [V, w] = follow.getRealVw(time); 
        end
        
        [vl, vr] = robotModel.VwTovlvr(V, w);
        
        count = count + 1;

        % robot.sendVelocity(vl, vr);
        lastT = currT;
        pause(0.05);
    end
    robot.stop()
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
plot(-refYArr(:, :, 1), refXArr(:, :, 1));
plot(-realY(:, :, 1), realX(:, :, 1));
legend({'Reference Path', 'Robot Path'});

figure(2)
clf;
hold on;
xlabel('Robot X (m)');
ylabel('Robot Y (m)');
title('Second Section');
plot(-refYArr(:, :, 2), refXArr(:, :, 2));
plot(-realY(:, :, 2), realX(:, :, 2));
legend({'Reference Path', 'Robot Path'});

figure(3)
clf;
hold on;
hold on;
xlabel('Robot X (m)');
ylabel('Robot Y (m)');
plot(-refYArr(:, :, 3), refXArr(:, :, 3));
plot(-realY(:, :, 3), realX(:, :, 3));
legend({'Reference Path', 'Robot Path'});