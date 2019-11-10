function laserEventListener(handle, event)
    global robot;
    global laserscan;
    global samescan;
 
    tempscan = robot.laser.LatestMessage.Ranges;
    if laserscan ~= tempscan
        laserscan = tempscan;
        samescan = false;
    else
        samescan = true;
    end
end


