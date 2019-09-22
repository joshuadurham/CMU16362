%% 3.1 Read and Display Lidar Data
dt = 0.2;
laserRanges = zeros(1,360);
xyThetas = zeros(3,360);

robot = raspbot('Raspbot-19');
robot.startLaser();
pause(2);
tic;
currTime = toc;
prevTime = currTime;
figure(1); clf;
while (true)
   prevTime = currTime;
   laserRanges = robot.laser.LatestMessage.Ranges;
   for i=1:360
        [x,y,th] = irToXy(i, laserRanges(i));
        xyThetas(:,i) = [x;y;th];
   end
   plotData(xyThetas, laserRanges)
   pause(0.2);
end
robot.stopLaser();

%% 3.2 Plot position of Nearest Object
dt = 0.2;
laserRanges = zeros(1,360);
xyThetas = zeros(3,360);

robot = raspbot('Raspbot-19');
robot.startLaser();
pause(2);
tic;
currTime = toc;
prevTime = currTime;
figure(1); clf;
while (true)
   prevTime = currTime;
   laserRanges = robot.laser.LatestMessage.Ranges;
   for i=1:360
        [x,y,th] = irToXy(i, laserRanges(i));
        xyThetas(:,i) = [x;y;th];
   end
   plotDataPart2(xyThetas, laserRanges)
   pause(0.2);
end
robot.stopLaser();

%% 3.3 Distance Servo
dt = 0.2;
laserRanges = zeros(1,360);
xyThetas = zeros(3,360);

robot = raspbot('Raspbot-19');
robot.startLaser();
pause(2);
tic;
currTime = toc;
prevTime = currTime;
figure(1); clf;
while (true)
   prevTime = currTime;
   laserRanges = robot.laser.LatestMessage.Ranges;
   for i=1:360
        [x,y,th] = irToXy(i, laserRanges(i));
        xyThetas(:,i) = [x;y;th];
   end
   [targetX, targetY, actualRange] = plotDataPart3(xyThetas, laserRanges)
   if (targetX == -1 && targetY == -1)
        robot.sendVelocity(0, 0);
   else
       targetDist = 0.5;
       kP = 1;
       distDiff = actualRange - targetDist;
       robot.sendVelocity(kP*distDiff, kP*distDiff);
   end
   pause(0.2);
end
robot.stopLaser();

%% 3.4 Challenge Task
clear all;
dt = 0.2;
laserRanges = zeros(1,360);
xyThetas = zeros(3,360);

robot = raspbot('Raspbot-22');
robot.startLaser();
pause(2);
tic;
currTime = toc;
prevTime = currTime;
figure(1); clf;
while (true)
   prevTime = currTime;
   laserRanges = robot.laser.LatestMessage.Ranges;
   for i=1:360
        [x,y,th] = irToXy(i, laserRanges(i));
        xyThetas(:,i) = [x;y;th];
   end
   [targetX, targetY, actualRange] = plotDataPart3(xyThetas, laserRanges)
   if (targetX == -1 && targetY == -1)
        robot.sendVelocity(0, 0);
   else
       wheelBase = 0.1;
       targetDist = 0.5;
       kP = 1;
       
       curvature = -2*targetY/(actualRange*actualRange);
       distDiff = actualRange - targetDist;
       velocity = kP*distDiff;
       omega = velocity*curvature;
       leftVel = velocity + (wheelBase*omega/2);
       rightVel = velocity - (wheelBase*omega/2);
       robot.sendVelocity(leftVel, rightVel);
   end
   pause(0.2);
end
robot.stopLaser();

function [] = plotData(xyThetaData, rangeData)
    plottedData = [];
    for i=1:size(xyThetaData,2)
        if rangeData(i) > 0 && rangeData(i) < 1.0
            plottedData(:, end+1) = xyThetaData(:, i);
        end
    end
   scatter(-plottedData(2,:), plottedData(1,:), 'x');
   xlim([-2 2])
   ylim([-2 2])
end

function [xCoord, yCoord, range] = plotDataPart3(xyThetaData, rangeData)
    plottedData = [];
    filteredRangeData = [];
    for i=1:size(xyThetaData,2)
        rangeDataIdx = i + 5;
        if rangeDataIdx > 360
            rangeDataIdx = rangeDataIdx - 360;
        end
        th = xyThetaData(3,i);
        if rangeData(rangeDataIdx) > 0.1 && rangeData(rangeDataIdx) < 1.0 && (th < 90 || th > 270)
            plottedData(:, end+1) = xyThetaData(:, i);
            filteredRangeData(end+1) = rangeData(rangeDataIdx);
        end
    end
    [range, minIdx] = min(filteredRangeData);
    minXYTh = plottedData(:, minIdx);
    xCoord = -1;
    yCoord = -1;
    if (size(minXYTh,1) > 1)
        display(minXYTh);
        xCoord = minXYTh(1);
        yCoord = minXYTh(2);
        scatter(-minXYTh(2), minXYTh(1), 'x');
        xlim([-2 2])
        ylim([-2 2])
        title('(x,y) coordinate of closest object in front of robot centered at origin (0,0)');
    xlabel('x coordinate of closest object in front of robot (m)');
    ylabel('y coordinate of closest object in front of robot (m)');
    legend({'closest object position'}, 'Location', 'southwest');

    end
end

function [] = plotDataPart2(xyThetaData, rangeData)
    plottedData = [];
    filteredRangeData = [];
    for i=1:size(xyThetaData,2)
        rangeDataIdx = i + 5;
        if rangeDataIdx > 360
            rangeDataIdx = rangeDataIdx - 360;
        end
        th = xyThetaData(3,i);
        if rangeData(rangeDataIdx) > 0.06 && rangeData(rangeDataIdx) < 1.0 && (th < 90 || th > 270)
            plottedData(:, end+1) = xyThetaData(:, i);
            filteredRangeData(end+1) = rangeData(rangeDataIdx);
        end
    end
    [~, minIdx] = min(filteredRangeData);
    minXYTh = plottedData(:, minIdx);
    if (size(minXYTh,1) > 1)
        display(minXYTh);
        scatter(-minXYTh(2), minXYTh(1), 'x');
        xlim([-2 2])
        ylim([-2 2])
    end
end

function [x,y,th] = irToXy(i, r)
    %finds position of pixel in plane
    th = i - 5;
    if (th < 1)
        th = th + 360;
    end
    x = r*cos(th*pi/180);
    y = r*sin(th*pi/180);
end