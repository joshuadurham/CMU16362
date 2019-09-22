function line_rth = ParametrizeABC2Rth(line_abc)
%PARAMETRIZEABC2RTH Converts a line in the form ax + by + c = 0 to a line
% in the form x*cos(theta) + y*sin(theta) = r.
%
%   line_rth = PARAMETRIZEABC2RTH(line_abc) takes a vector of parameters
%   [a; b; c] that characterize a line in the form ax + by + c = 0 and
%   returns a vector of parameters [r; theta] that characterizes the same
%   line in the form x*cos(theta) + y*sin(theta) = r.

a = line_abc(1); b = line_abc(2);
c = line_abc(3);

x = -(a*c)/(a^2+b^2);
y = -(b*c)/(a^2+b^2);
r = norm([x y]);
th = atan2(y,x);

line_rth = [r;th];
end

