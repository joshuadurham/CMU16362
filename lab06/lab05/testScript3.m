%%3.3 test script
b = figure8(3, 1, 0.5);
t = robotTrajectory(b, 0, 0, 0, 0, 1000, 0, b.tf + 2 * b.tPause);
x = t.poseSamples(1, :);
y = t.poseSamples(2, :);
plot(x, y);