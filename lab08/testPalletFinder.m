load('TestLasers5.mat', 'laserData');
mrpl = mrplSystem();
laserObj = rangeImage(laserData, 1, true);
[xf, yf, thf, errf, numf] = mrpl.findSail(laserObj);
scatter(laserObj.xArray, laserObj.yArray);
hold on
disp(numf);
scatter(xf, yf, 100);
