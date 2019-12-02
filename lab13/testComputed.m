load('smallMotions.mat')
% backup or turnAround
dur = turnAround.getTrajectoryDuration();

currT = 0;
firstIteration = false;
vArr = zeros(1, 1000);
wArr = zeros(1, 1000);
xArr = zeros(1, 1000);
thArr = zeros(1, 1000);
count = 1;
lastT = 0;
while(currT < dur)
    if(firstIteration == false)
        startTic = tic();        
        firstIteration = true;
        continue;
    end
    
    currT = toc(startTic);
    dt = currT - lastT;
     
    V = turnAround.getVAtTime(currT);
    w = turnAround.getwAtTime(currT);
    vArr(count) = V;
    wArr(count) = w;
    if count > 1
        thArr(count) = w * dt + thArr(count-1);
    else
        thArr(count) = 0;
    end
    lastT = currT;
    count = count + 1;
    pause(0.05)
end


    