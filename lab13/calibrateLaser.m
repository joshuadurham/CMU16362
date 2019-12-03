% calibrate robot
rb = raspbot('RaspBot-11');
rb.startLaser();
pause(5);
laserData = rb.laser.LatestMessage.Ranges;

sailFinder = rangeImage(laserData, 1, false, false);
clf;
sailFinder.plotXvsY(2);
rb.stopLaser();
rb.forksUp();

xs = sailFinder.xArray;
ys = sailFinder.yArray;

rb.shutdown();