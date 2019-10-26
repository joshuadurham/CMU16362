% calibrate robot
rb = raspbot('RaspBot-16');
rb.startLaser();
pause(5);
laserData = rb.laser.LatestMessage.Ranges;

sailFinder = rangeImage(laserData, 1, true);
clf;
sailFinder.plotXvsY(2);
rb.stopLaser();
rb.forksUp();

rb.shutdown();