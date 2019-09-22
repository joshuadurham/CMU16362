%% Task 3.1 The Robot Listener
global prevTimestamp;
global prevLeftEncoder;
global prevRightEncoder;
global x;
global y;
global theta;
global robot;
global positionIdx;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;
x = 0;
y = 0;
theta = 0;
robot = raspbot('Raspbot-19');
positionIdx = 1;
robot.encoders.NewMessageFcn=@encoderEventListener;
while(true)
    display(x);
    display(y);
    display(theta);
    display(positionIdx);
    pause(0.25);
end

%% Task 3.3, Simulation for Arbitrary Trajectories
[xRes,yRes,thetaRes] = modelDiffSteerRobot(0, 0, 0, 15, 0.001);
plot(xRes, yRes)
axis equal
xlim([0, 0.4])
ylim([0, 0.4])

%% Task 3.3.3 Figure 8 estimation
vOft = 0.2;
sf = 1;
tf = sf/vOft;
kTheta = 2*pi/sf;
kk = 15.1084;
ks = 3;
Tf = ks*tf;
wheelBase = 0.1;

xArray = zeros(1,10000);
yArray = zeros(1,10000);
thetaArray = 0;
initTic = tic();
prevt = 0;
prevs = 0;

myPlot = plot(xArray, yArray, 'b-');
xlim([-0.5, 0.5]);
ylim([-0.5, 0.5]);
idx = 2;
while (toc(initTic) < Tf)
    currT = toc(initTic);
    unscaledt = currT/ks;
    sOft = vOft*unscaledt;
    curvOft = (kk/ks)*sin(kTheta*sOft);
    omegaOft = curvOft*vOft;
    
    deltaS = sOft - prevs;
    deltaT = unscaledt - prevt;
    vl = vOft + wheelBase/2*omegaOft;
    vr = vOft - wheelBase/2*omegaOft;
    newTheta = thetaArray(idx-1) + ks*omegaOft*deltaT/2;
    xArray(idx) = xArray(idx-1) + deltaS*cos(newTheta);
    yArray(idx) = yArray(idx-1) + deltaS*sin(newTheta);
    thetaArray(idx) = newTheta + ks*omegaOft*deltaT/2;
    prevt = unscaledt;
    prevs = sOft;
    set(myPlot, 'xdata', [get(myPlot,'xdata') xArray(idx)], ...
        'ydata', [get(myPlot,'ydata') yArray(idx)]);
    idx = idx + 1;
    pause(0.005);
end
plot(xArray, yArray);
    

%% Task Challenge Task: Drive and Estimate Pose
clear all;
global prevTimestamp;
global prevLeftEncoder;
global prevRightEncoder;
global x;
global y;
global theta;
global robot;
global positionIdx;
global initLeftEncoder;
global initRightEncoder;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;
x = 0;
y = 0;
theta = 0;
robot = raspbot('Raspbot-19');
positionIdx = 1;
robot.encoders.NewMessageFcn=@encoderEventListener;
initLeftEncoder = robot.encoders.LatestMessage.Vector.X;
initRightEncoder = robot.encoders.LatestMessage.Vector.Y;

vOft = 0.2;
sf = 1;
tf = sf/vOft;
kTheta = 2*pi/sf;
kk = 15.1084;
ks = 3;
Tf = ks*tf;
wheelBase = 0.089;

xArray = zeros(1,10000);
yArray = zeros(1,10000);
thetaArray = zeros(1,10000);
initTic = tic();
prevt = 0;
prevs = 0;

myPlot = plot(x, y, 'b-');
xlim([-0.8 0.8]);
ylim([-0.8 0.8]);
title('(x,y) coordinate of robot, initial position at origin (0,0)');
xlabel('x coordinate of robot (m)');
ylabel('y coordinate of robot (m)');
%legend({'path of robot position in (x,y) coordinates'}, 'Location', 'southwest');
idx = 2;
while (toc(initTic) < Tf)
    currT = toc(initTic);
    unscaledt = currT/ks;
    sOft = vOft*unscaledt;
    curvOft = (kk/ks)*sin(kTheta*sOft);
    omegaOft = curvOft*vOft;
    
    deltaS = (sOft - prevs)*ks;
    deltaT = unscaledt - prevt;
    vl = vOft + wheelBase/2*omegaOft;
    vr = vOft - wheelBase/2*omegaOft;
    newTheta = thetaArray(idx-1) + ks*omegaOft*deltaT/2;
    xArray(idx) = xArray(idx-1) + deltaS*cos(newTheta);
    yArray(idx) = yArray(idx-1) + deltaS*sin(newTheta);
    thetaArray(idx) = newTheta + ks*omegaOft*deltaT/2;
    prevt = unscaledt;
    prevs = sOft;
    set(myPlot, 'xdata', [get(myPlot,'xdata') x], ...
        'ydata', [get(myPlot,'ydata') y]);
    idx = idx + 1;
    robot.sendVelocity(vr, vl);
    pause(0.005);
end
robot.sendVelocity(0.2,0.2)
pause(0.55);
robot.stop();
robot.shutdown();

function encoderEventListener(handle, event)
    wheelbase = 0.089;
    global prevTimestamp;
    global prevLeftEncoder;
    global prevRightEncoder;
    global x;
    global y;
    global theta;
    global robot;
    global positionIdx;
    global initLeftEncoder;
    global initRightEncoder;
    
    currTimestamp = double(event.Header.Stamp.Sec) + ...
        double(event.Header.Stamp.Nsec)/1000000000.0;
    currLeftEncoder = robot.encoders.LatestMessage.Vector.X;
    currRightEncoder = robot.encoders.LatestMessage.Vector.Y;
    currLeftEncoder = currLeftEncoder - initLeftEncoder;
    currRightEncoder = currRightEncoder - initRightEncoder;
    leftEncoderDiff = currLeftEncoder - prevLeftEncoder;
    rightEncoderDiff = currRightEncoder - prevRightEncoder;
    
    prevTimestamp = currTimestamp;
    prevLeftEncoder = currLeftEncoder;
    prevRightEncoder = currRightEncoder;
    
    thetaChange = atan((rightEncoderDiff - leftEncoderDiff)/wheelbase);
    positionChange = (leftEncoderDiff + rightEncoderDiff)/2;
    theta = theta + thetaChange/2;
    x = x + positionChange*cos(theta);
    y = y + positionChange*sin(theta);
    theta = theta + thetaChange/2;
    positionIdx = positionIdx + 1;
    %plot(x,y);
end

function [x, y, th] = modelDiffSteerRobot(vl, vr, t0, tf, dt)
    wheelBase = 0.1;
    t = t0:dt:tf;
    display(t);
    display(size(t));
    x = zeros(size(t));
    y = zeros(size(t));
    th = zeros(size(t));
    for i=2:size(t,2)
        % FOR CORNU SPIRAL
        %v = 0.1;
        %omega = 0.125*t(i);
        %vr = v + wheelBase*omega/2;
        %vl = v - wheelBase*omega/2;        
        bodyVel = (vl + vr)/2;
        omega = (vr - vl)/wheelBase;
        deltaS = bodyVel * dt;
        deltaTheta = omega * dt;
        newTheta = th(i-1) + deltaTheta/2;
        x(i) = x(i-1) + deltaS*cos(newTheta);
        y(i) = y(i-1) + deltaS*sin(newTheta);
        th(i) = newTheta + deltaTheta/2;
    end
end