function [] = encoderListener(handle,event)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global left
global right
global encFrame
global timeStamp
% disp(event);
% pseudocode:
% while (data is the same)
%   pause(1 ms)
% end
% timeStamp = newTime
% frame++
% rightData = newRight
% leftData = newLeft
%% reAL
left = event.Vector.X;
right = event.Vector.Y;

timeStamp = double(event.Header.Stamp.Sec) + double(event.Header.Stamp.Nsec)/ 1000000000.0;
encFrame = encFrame + 1;
end

