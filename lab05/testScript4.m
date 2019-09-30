a = trapezoidalReferenceControl(0.75, 0.25, 1, 1, 0);
f = robotTrajectory(a, 0, 0, 0, 0, 1000, 0, a.tf);

robot = raspbot('sim', [0;0;0]);

pause(2);
currT = 0;
prevT = 0;
firstIteration = false;
vla = zeros(1, 1000);
vra = zeros(1, 1000);
count = 1;
while(currT < 6)
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
    end
    currT = toc(startTic);
    [vl, vr] = f.getVlVrAtT(currT);
    vla(count) = vl;
    vlr(count) = vr;
    count = count + 1;
    robot.sendVelocity(vl, vr);
    pause(0.05);
end
