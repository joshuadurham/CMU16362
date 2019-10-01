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

wheelbase = 0.09;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;
x = 0;
y = 0;
theta = 0;

robot = raspbot('RaspBot-19');
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

feedBack = false;

count = 1;
pause(3);
lastT = 0;
thReal = 0;
yReal = 0;
xReal = 0;

while(currT < t.ref.getTrajectoryDuration())
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
    end
    currT = toc(startTic);
    
    xReal = x;
    yReal = y;
    thReal = theta;
    
    time = currT / t.ref.ks;
    
    [vl, vr] = t.getVlVrAtT(time);
    [v, w] = robotModel.vlvrToVw(vl, vr);
    
    vl = vl / b.ks * 1.15;
    vr = vr / b.ks * 1.15;
    
    dt = time - lastT;
    
    if (feedBack)
    end
        
    realX(positionIdx) = xReal;
    realY(positionIdx) = yReal;
    realTh(positionIdx) = thReal;
     
    [refx, refy, refth] = t.getPoseAtT(time);
    refXArr(positionIdx) = refx;
    refYArr(positionIdx) = refy;
    refThArr(positionIdx) = refth;
    
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
pause(1);
robot.stop();
refXArr = refXArr(1:positionIdx-1);
refYArr = refYArr(1:positionIdx-1);
refThArr = refThArr(1:positionIdx-1);
realX = realX(1:positionIdx-1);
realY = realY(1:positionIdx-1);
realTh = realTh(1:positionIdx-1);
robot.shutdown();

