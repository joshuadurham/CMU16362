clear all;
clf;
% xarr = zeros(1, 1000);
% yarr = zeros(1, 1000);
% tharr = zeros(1, 1000);
% 
% % myplot = plot(xarr, yarr, 'b-');
% xlim([-0.6 0.6]);
% ylim([-0.6 0.6]);
% title('Path of Robot');
% xlabel('x (meters)');
% ylabel('y (meters)');
% 
% Varr = zeros(1, 1000);
% warr = zeros(1, 1000);
% refXArr = zeros(1, 1000);
% refYArr = zeros(1, 1000);
% vla = zeros(1, 1000);
% vra = zeros(1, 1000);


params = [0.5, 0.0, 0.0];

 % change this for ease of use


Vref = 0.2;

x = 0;
y = 0;
theta = 0;

robot = raspbot('sim', [0;0;0]);

count = 1;

xf = 0.5;
yf = 0;
thf = 0;
sgn = 1;

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
    time = currT;
    V = spline.getVAtTime(time);
    w = spline.getwAtTime(time);

    pose = spline.getPoseAtTime(time);
    refx = pose(1);
    refy = pose(2);

    [vl, vr] = robotModel.VwTovlvr(V, w);
    theta = theta + w * dt /2;
    x = x + V * dt * cos(theta);
    y = y + V * dt * sin(theta);
    theta = theta + w * dt /2;

%     xarr(count) = x;
%     yarr(count) = y;
%     tharr(count) = theta;
%     Varr(count) = V;
%     warr(count) = w;
%     refXArr(count) = refx;
%     refYArr(count) = refy;
%     vla(count) = vl;
%     vra(count) = vr;

    count = count + 1;

    robot.sendVelocity(vl, vr);
    % set(myplot, 'xdata', [get(myplot, 'xdata') x], 'ydata', [get(myplot, 'ydata') y])
    pause(0.05);
end

robot.stop();
robot.shutdown();
% plot(xarr(1:count-1), yarr(1:count-1));