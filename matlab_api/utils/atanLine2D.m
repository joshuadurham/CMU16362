function theta = atanLine2D(a,b)
%ATANLINE2D Returns the angle of the line of form ax + by + c = 0.
%
%   theta = ATANLINE2D takes a and b from the line equation ax + by + c = 0
%   and returns the angle of the line, theta.
theta = atan2(-a,b);
if a > 0
    theta = pi + theta;
end
if a == 0
    theta = 0;
end
if b == 0
    theta = pi/2;
end
end

