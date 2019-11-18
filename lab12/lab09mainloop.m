%% INTERNAL LOOP TO RUN LAB 09, do not touch here. Left as a reminder of how it should be run with the new code improvemnts.
% returns Pgr, pose of goal to robot (not the actual
% pallet)
[xf, yf, thf] = obj.getSailPosition()
% if nothing was found, redo it
% if xf == 10000
%     continue
% end
% plan and run the trajectory to the acquisition pose
obj = obj.runRobot(tau, largeMotionFeedBack, [xf, yf, thf], false, false, 1);
pause(1);

[xf, yf, thf] = obj.getSailPosition();

Pgr = pose(xf, yf, thf);
% transform of pallet to robot 
Tpr = Pgr.bToA / obj.Tgp;
% transform of ideal final (distance + 5 cm) to robot
Tfr = Tpr * obj.Tfp;
Pfr = pose.matToPoseVec(Tfr);
xf = Pfr(1);
yf = Pfr(2);
thf = Pfr(3);
obj = obj.runRobot(tau, largeMotionFeedBack, [xf, yf, thf], false, false, 1);

pause(1);
obj = obj.smallMotions(tau, smallMotionFeedBack);
pause(10);
i = i + 1;