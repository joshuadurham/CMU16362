function [m,c] = ParametrizePts2MC(p1,p2)
%PARAMETRIZEPTS2MC Returns the line that joins two points in the form y =
% mx + b.
%
%   [m, c] = PARAMETRIZEPTS2MC(p1, p2) takes the points p1 and p2 and
%   returns the values m and c of line that joins them in the form y = mx +
%   c.

m = (p2(2)-p1(2))/(p2(1)-p1(1));
c = p1(2)-m*p1(1);

end