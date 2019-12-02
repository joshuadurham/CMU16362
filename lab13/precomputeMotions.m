backup = trapezoidalReferenceControl(0.1, 0.2, 0.05, -1, 0);
turnAround = rotationalReferenceControl(0.1, 0.2, pi(), 1, 0);
save('smallMotions.mat', 'backup', 'turnAround');
