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
    global frames
    
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
    frames = frames + 1;
    % positionIdx = positionIdx + 1;
end
