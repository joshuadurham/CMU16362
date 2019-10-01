b = figure8(3, 1, 0);
t = robotTrajectory(b, 0, 0, 0, 0, 1000, 0, b.tf + 2 * b.tPause);

robot = raspbot('sim', [0; 0; 0]);

currT = 0;
prevT = 0;
firstIteration = false;

vla = zeros(1, 1000);
vra = zeros(1, 1000);
tarr = zeros(1, 1000);
xarr = zeros(1, 1000);
yarr = zeros(1, 1000);
tharr = zeros(1, 1000);
realX = zeros(1, 1000);
realY = zeros(1, 1000);
realTh = zeros(1, 1000);

count = 1;
pause(2);
lastT = 0;
thReal = 0;
yReal = 0;
xReal = 0;
while(currT < b.getTrajectoryDuration())
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
    end
    currT = toc(startTic);
    time = currT / t.ref.ks;
    
    [vl, vr] = t.getVlVrAtT(time);
    [v, w] = robotModel.vlvrToVw(vl, vr);
    
%     vl = vl;
%     vr = vr * 1.1458;
    
    dt = time - lastT;
    
    thReal = thReal + t.ref.ks * w ./2 * dt;
    xReal = xReal + v * 1.1458 * dt * cos(thReal);
    yReal = yReal + v * 1.1458 * dt * sin(thReal);
    thReal = thReal +  t.ref.ks * w ./2 * dt;
    realX(count) = xReal;
    realY(count) = yReal;
    realTh(count) = thReal;
     
    [x, y, th] = t.getPoseAtT(time);
    xarr(count) = x;
    yarr(count) = y;
    tharr(count) = th;
    
    if (isnan(vl))
        vl = 0;
    end
    if (isnan(vr))
        vr = 0;
    end
    
    vla(count) = vl;
    vra(count) = vr;
    tarr(count) = time;
    
    count = count + 1;
    robot.sendVelocity(vl, vr);
%     s = v .* dt + slast;
    lastT = time;
    pause(0.05);
end
robot.stop();
xarr = xarr(1:count-1);
yarr=yarr(1:count-1);
tharr = tharr(1:count-1);
realX = realX(1:count-1);
realY = realY(1:count-1);
realTh = realTh(1:count-1);
plot(realX, realY);
robot.shutdown();