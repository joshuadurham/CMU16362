endpoints = [0,      0,      0;
                     0.3048, 0.3048, 0.0;
                    -0.3048, -0.3048, - pi()/2.0;
                    -0.6096, 0, 0];
            
i = 3;
origin = endpoints(i-1, :);
ox = origin(1);
oy = origin(2);
oth = origin(3);
originPose = pose(ox, oy, oth);

Tow = originPose.aToB();

nextPoint = endpoints(i, :);
nextX = nextPoint(1);
nextY = nextPoint(2);
nextTh = nextPoint(3);
nextPose = pose(nextX, nextY, nextTh);

Tew = nextPose.bToA();

Toe = Tow * Tew;

endToRobotPose = pose.matToPoseVec(Toe);
xf = endToRobotPose(1)
yf = endToRobotPose(2)
thf = endToRobotPose(3)