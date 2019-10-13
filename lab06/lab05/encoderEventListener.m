function encoderEventListener(handle, event)
    global prevTimestamp;
    global prevLeftEncoder;
    global prevRightEncoder;
    global robot;
    global initLeftEncoder;
    global initRightEncoder;
    global leftEncoderDiff;
    global rightEncoderDiff;
 
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
    
end
