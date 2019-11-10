function encoderEventListener(handle, event)
    global timestamp;
    global robot;
    global currLeftEncoder;
    global currRightEncoder;
    global frames;
 
    currTimestamp = double(event.Header.Stamp.Sec) + ...
        double(event.Header.Stamp.Nsec)/1000000000.0;
    currLeftEncoder = robot.encoders.LatestMessage.Vector.X;
    currRightEncoder = robot.encoders.LatestMessage.Vector.Y;
    
    timestamp = currTimestamp;
    frames = frames + 1;
end
