clear all;
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
b = figure8(3, 1, 0);
t = robotTrajectory(b, 0, 0, 0, 0, 1000, 0, b.tf + 2 * b.tPause);

positionIdx = 1;
currT = 0;
prevT = 0;
firstIteration = false;

vla = zeros(1, 1000);
vra = zeros(1, 1000);
tarr = zeros(1, 1000);
refXArr = zeros(1, 1000);
refYArr = zeros(1, 1000);
refThArr = zeros(1, 1000);
realX = zeros(1, 1000);
realY = zeros(1, 1000);
realTh = zeros(1, 1000);
tau = 85;

feedBack = true;

count = 1;
pause(3);
lastT = 0;
thReal = 0;
yReal = 0;
xReal = 0;

Vref = 0.2;
control = 0;
acPose = 0;
refPose = 0;
follow = 0;

dur = t.ref.getTrajectoryDuration();
ks = t.ref.ks;

while(frames < 5)
    pause(0.05);
end

robot.sendVelocity(0, 0);

while(currT < dur + 1)
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
        continue;
    end
    
    currT = toc(startTic);
    time = currT ./ ks;
    
    [vl, vr] = t.getVlVrAtT(time);
    
    if (currT > dur)
        vl = 0;
        vr = 0;
        tau = 0.7308;
    else
        [refx, refy, refth] = t.getPoseAtT(time);
    end
    [v, w] = robotModel.vlvrToVw(vl, vr);
    
    dt = time - lastT;
    
    
    refXArr(positionIdx) = refx;
    refYArr(positionIdx) = refy;
    refThArr(positionIdx) = refth;
    
    xReal = x;
    yReal = y;
    thReal = theta;
    
    if (feedBack)
        refPose = pose(refx, refy, refth);
        acPose = pose(xReal, yReal, thReal);
        control = controller(refPose, acPose, Vref, tau);
        follow = trajectoryFollower(t, control);
        [v, w] = follow.getRealVw(time); 
    end
    
    [vl, vr] = robotModel.VwTovlvr(v, w);
    
    realX(positionIdx) = xReal;
    realY(positionIdx) = yReal;
    realTh(positionIdx) = thReal;
     
    if (isnan(vl))
        vl = 0;
    end
    if (isnan(vr))
        vr = 0;
    end
    
    vla(positionIdx) = vl;
    vra(positionIdx) = vr;
    tarr(positionIdx) = time;
    
    positionIdx = positionIdx + 1;
    robot.sendVelocity(vl, vr);
    
    pause(0.05);
end
pause(0.5);
robot.stop();
robot.shutdown();
refXArr = refXArr(1:positionIdx-1);
refYArr = refYArr(1:positionIdx-1);
refThArr = refThArr(1:positionIdx-1);
realX = realX(1:positionIdx-1);
realY = realY(1:positionIdx-1);
realTh = realTh(1:positionIdx-1);
tarr = tarr(1:positionIdx-1);
vla = vla(1:positionIdx-1);
vra = vra(1:positionIdx-1);

figure(1)
clf;
hold on;
xlabel('Time (seconds)');
ylabel('Error (meters)');
errX = refXArr - realX;
errY = refYArr - realY;
errTh = refThArr - realTh;
plot(tarr, errX);
plot(tarr, errY);
plot(tarr, errTh);
legend({'Error X', 'Error Y', 'Error Theta'});

figure(2)
clf;
hold on;
xlabel('Robot X Coordinates (m)');
ylabel('Robot Y Coordinates (m)');
plot(-refYArr, refXArr);
plot(-realY, realX);
legend({'Reference Path', 'Robot Path'});

figure(3)
clf;
hold on;
xlabel('Time (seconds)');
ylabel('Output');
plot(tarr, refXArr);
plot(tarr, refYArr);
plot(tarr, refThArr);
plot(tarr, realX);
plot(tarr, realY);
plot(tarr, realTh);
legend({'Ref X', 'Ref Y', 'Ref Th', 'Real X', 'Real Y', 'Real Th'}, 'NumColumns', 2);