clear all;
global wheelbase;
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
wheelbase = 0.09;
prevTimestamp = 0;
prevLeftEncoder = 0;
prevRightEncoder = 0;
x = 0;
y = 0;
theta = 0;
% robot = raspbot('sim');
% other robot
robot = raspbot('RaspBot-19');
pause(3);
positionIdx = 1;
robot.encoders.NewMessageFcn=@encoderEventListener;
initLeftEncoder = robot.encoders.LatestMessage.Vector.X;
initRightEncoder = robot.encoders.LatestMessage.Vector.Y;

% ADJUST THESE HERE
useFeedback = false;
timeDelay = 0.4;
kp = 0.5;
ki = 0;
kd = 0;

goalPosition = 1;
vmax = 0.25;
amax = 3 * vmax;

prevError = 0;
accError = 0;
feedForward = 0;
currIdealPos = 0;
currT = 0;
prevT = 0;

feedForward = 0;
delayedFeedForward = 0;
currIdealPos = 0;
delayedCurrIdealPos = 0;
firstIteration = false;
scaleFactor = 1;

% Graphing setup
len = 10000;
idealArr = zeros(1, len);
realArr = zeros(1, len);
errArr = zeros(1, len);
tArr = zeros(1, len);
vArr = zeros(1, len);

myplot = plot(errArr, tArr, 'b-');
count = 1;
realX = x;
error = currIdealPos - realX;
while((abs(goalPosition - realX) > 0.001))
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
    end
    currT = toc(startTic);
    deltaT = currT - prevT;
    currIdealPos = currIdealPos + feedForward * deltaT;
    if (feedForward == 0 && currT > 3) 
       currIdealPos = goalPosition;
    end
    delayedCurrIdealPos = delayedCurrIdealPos + delayedFeedForward * deltaT;
    error = currIdealPos - realX;
    errorDelta = (error - prevError)/deltaT;
    accError = accError + error*deltaT;
    cmd = kp*error + ki*accError + kd*errorDelta;
    if (cmd > 0.3)
        cmd = 0.3;
    end
    feedForward = trapezoidalVelocityProfile(currT, amax, vmax, goalPosition, 1) * scaleFactor;
    delayedFeedForward = trapezoidalVelocityProfile(currT - timeDelay, amax, vmax, goalPosition, 1);
    if (useFeedback) 
        robot.sendVelocity(cmd + feedForward, cmd + feedForward);
    else
        robot.sendVelocity(feedForward, feedForward);
    end
    prevT = currT;
    prevError = error;
%     plot
%     set(myplot, 'xdata', [get(myplot, 'xdata') currT], 'ydata', [get(myplot, 'ydata') currIdealPos])
%     set(myplot, 'xdata', [get(myplot, 'xdata') currT], 'ydata', [get(myplot, 'ydata') currIdealPos])
    idealArr(count) = currIdealPos;
    realArr(count) = realX;
    tArr(count) = currT;
    vArr(count) = feedForward;
    count = count + 1;
    realX = x;
    if(currT >= 18)
        break;
    end
    pause(0.01);
end

% while(currT < 5)
%     if(firstIteration == false)
%         startTic = tic();    
%         firstIteration = true;
%     end
%     currT = toc(startTic);
%     feedForward = trapezoidalVelocityProfile(currT, amax, vmax, goalPosition, 1) * scaleFactor;
%     robot.sendVelocity(feedForward, feedForward);
%     tArr(count) = currT;
%     vArr(count) = feedForward;
%     count = count + 1;
% end

robot.stop();
robot.shutdown();
clf;
hold on;
plot(tArr(1:count-1), realArr(1:count-1));
plot(tArr(1:count-1), idealArr(1:count-1));
plot(tArr(1:count-1), vArr(1:count-1));

legend('real x', 'ideal x', 'velocity');

function uref = trapezoidalVelocityProfile(t, amax, vmax, dist, sgn)
    tRamp = vmax / amax;
    tF = (dist + (vmax*vmax)/amax)/vmax;
    if (t < 0 || t > tF)
        uref = 0;
    elseif (t < tRamp)
        uref = amax * t;
    elseif (tF - t < tRamp)
        uref = amax * (tF - t);
    elseif (tRamp < t && t < tF - tRamp)
        uref = vmax;
    else 
        uref = 0;
    end
    uref = sgn * uref;
end

function encoderEventListener(handle, event)
    global wheelbase;
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