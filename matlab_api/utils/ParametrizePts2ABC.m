function params = ParametrizePts2ABC(p1,p2)
%PARAMETRIZEPTS2ABC Returns the line that joins two points in the form ax +
% by + c = 0.
%
%   params = PARAMETRIZEPTS2ABC(p1, p2) takes the points p1 and p2 and
%   returns the vector of parameters params = [a; b; c] of the line that
%   joins them in the form ax + by + c = 0.

if p1(1) == p2(1)
    params = [1;0;-p1(1)];
elseif p1(2) == p2(2)
    params = [0;1;-p1(2)];
else
    [m,c] = ParametrizePts2MC(p1,p2);
    params = [m;-1;c];
end

end

