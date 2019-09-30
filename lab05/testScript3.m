%%3.3 test script
b = figure8(3, 1, 0.5);
t = robotTrajectory(b, 0, 0, 1000);
x = t.poseSamples(1, :);
y = t.poseSamples(2, :);
plot(x, y);