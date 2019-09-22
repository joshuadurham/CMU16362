%% Lab 1 Task 1 Move the Robot
robot = raspbot('Raspbot-19');
tic;
currTime = toc;
while (currTime < 4)
   robot.sendVelocity(0.05, 0.05);
   pause(0.05);
   currTime = toc;
end
robot.sendVelocity(0,0);
pause(0.05);
tic;
currTime = toc;
while (currTime < 4)
   robot.sendVelocity(-0.05, -0.05);
   pause(0.05);
   currTime = toc;
end
robot.sendVelocity(0,0);

%% Lab 1 Task 2 Basic Simulation
leftStart = 6934;
rightStart = 4396;
currLeft = leftStart;
currRight = rightStart;
signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
vel = 0.05;

tic;
currTime = toc;
while (signedDistance < 12 * 2.54 / 100)
   currLeft = currLeft + vel*(toc - currTime);
   currRight = currRight + vel*(toc - currTime);
   
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2
   currTime = toc;
   pause(0.05);
end

%% Lab 1 Task 3 Basic Debugging
% I spent time debugging...

%% Lab 1 Task 4 Basic Plotting and Real Time Plotting
timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);

leftStart = 6934;
rightStart = 4396;
currLeft = leftStart;
currRight = rightStart;
signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
vel = 0.05;

tic;
currTime = toc;
while (signedDistance < 12 * 2.54 / 100)
   currLeft = currLeft + vel*(toc - currTime);
   currRight = currRight + vel*(toc - currTime);
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2
   currTime = toc;
   timeArray(end + 1) = currTime;
   leftArray(end + 1) = currLeft - leftStart;
   rightArray(end + 1) = currRight - rightStart;
   pause(0.05);
end
plot(timeArray, leftArray, timeArray, rightArray);

%% Lab 1 Task 5 Challenge Task
vel = 0.02;
robot = raspbot('Raspbot-18');
timeArray = zeros(1,1);
leftArray = zeros(1,1);
rightArray = zeros(1,1);

leftStart = robot.encoders.LatestMessage.Vector.X;
rightStart = robot.encoders.LatestMessage.Vector.Y;
currLeft = leftStart;
currRight = rightStart;
signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
tic;
figure
hold on
%while (signedDistance < 11.5 * 2.54 / 100)
while (signedDistance < 12 * 2.54 / 100)

   robot.sendVelocity(vel, vel);
   currLeft = robot.encoders.LatestMessage.Vector.X;
   currRight = robot.encoders.LatestMessage.Vector.Y;
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
   currTime = toc;
   timeArray(end + 1) = currTime;
   leftArray(end + 1) = (currLeft - leftStart) * 100;
   rightArray(end + 1) = (currRight - rightStart) * 100;
   plot(timeArray, leftArray, 'r', timeArray, rightArray, 'b');
   title('Encoder position (cm) versus time (s)');
   xlabel('seconds from start (s)');
   ylabel('encoder position (cm)');
   legend({'left encoder (cm)', 'right encoder (cm)'}, 'Location', 'southwest');
   pause(0.05);
end
startWaitTime = toc;
currTime = startWaitTime;
while (currTime - startWaitTime < 1)
    robot.sendVelocity(0, 0);
   currLeft = robot.encoders.LatestMessage.Vector.X;
   currRight = robot.encoders.LatestMessage.Vector.Y;
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
   currTime = toc;
   timeArray(end + 1) = currTime;
   leftArray(end + 1) = (currLeft - leftStart) * 100;
   rightArray(end + 1) = (currRight - rightStart) * 100;
   plot(timeArray, leftArray, 'r', timeArray, rightArray, 'b');
   title('Encoder position (cm) versus time (s)');
   xlabel('seconds from start (s)');
   ylabel('encoder position (cm)');
   legend({'left encoder (cm)', 'right encoder (cm)'}, 'Location', 'southwest');
   pause(0.05);
end
%while (signedDistance >  0.5 * 2.54 / 100)
while (signedDistance >  1.5 * 2.54 / 100)
   robot.sendVelocity(-vel, -vel);
   currLeft = robot.encoders.LatestMessage.Vector.X;
   currRight = robot.encoders.LatestMessage.Vector.Y;
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
   currTime = toc;
   timeArray(end + 1) = currTime;
   leftArray(end + 1) = (currLeft - leftStart) * 100;
   rightArray(end + 1) = (currRight - rightStart) * 100;
   plot(timeArray, leftArray, 'r', timeArray, rightArray, 'b');
   title('Encoder position (cm) versus time (s)');
   xlabel('seconds from start (s)');
   ylabel('encoder position (cm)');
   legend({'left encoder (cm)', 'right encoder (cm)'}, 'Location', 'southwest');
   pause(0.05);
end
currLeft = robot.encoders.LatestMessage.Vector.X;
currRight = robot.encoders.LatestMessage.Vector.Y;
signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
currTime = toc;
timeArray(end + 1) = currTime;
leftArray(end + 1) = (currLeft - leftStart) * 100;
rightArray(end + 1) = (currRight - rightStart) * 100;
plot(timeArray, leftArray, 'r', timeArray, rightArray, 'b');
title('Encoder position (cm) versus time (s)');
xlabel('seconds from start (s)');
ylabel('encoder position (cm)');
legend({'left encoder (cm)', 'right encoder (cm)'}, 'Location', 'southwest');

startWaitTime = toc;
currTime = startWaitTime;
while (currTime - startWaitTime < 1)
    robot.sendVelocity(0, 0);
   currLeft = robot.encoders.LatestMessage.Vector.X;
   currRight = robot.encoders.LatestMessage.Vector.Y;
   signedDistance = ((currLeft - leftStart) + (currRight - rightStart))/2;
   currTime = toc;
   pause(0.05);
end

