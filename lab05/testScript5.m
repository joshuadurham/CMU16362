b = figure8(3, 1, 0.5);
t = robotTrajectory(b, 0, 0, 0, 0, 1000, 0, b.tf + 2 * b.tPause);

robot = raspbot('sim', [0;0;0]);

currT = 0;
prevT = 0;
firstIteration = false;
vla = zeros(1, 1000);
vra = zeros(1, 1000);
tarr = zeros(1, 1000);
count = 1;
pause(2);
while(currT < 6)
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
    end
    currT = toc(startTic);
    time = currT;
    [vl, vr] = t.getVlVrAtT(time);
    vla(count) = vl;
    vra(count) = vr;
    tarr(count) = time;
    count = count + 1;
    robot.sendVelocity(vl, vr);
%     s = v .* dt + slast;
    pause(0.005);
end
robot.shutdown();