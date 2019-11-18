function [theta] = angleArith(ang1,ang2,sgn)
%UNTITLED Summary of this function goes here
%   sgn = 1 then add
%   sgn = -1 then subtract
    if sgn > 0 
        theta = atan2(sin(ang1 + ang2), cos(ang1 + ang2));
    else
        theta = atan2(sin(ang2 - ang1), cos(ang2 - ang1));
    end
        
end

