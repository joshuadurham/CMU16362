robbit = mrplSystem();
[tx, ty, cx, cy, xs, ys] = robbit.rangeTesting();

testLams = rangeImage.getEigenvalues(tx, ty);
realLams = rangeImage.getEigenvalues(cx, cy);
clf;
hold on
scatter(xs, ys);
scatter(tx, ty);