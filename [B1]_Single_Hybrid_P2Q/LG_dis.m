function [ dis ] = LG_dis( xfree )
%LG_DIS Summary of this function goes here
%   Detailed explanation goes here
xQ = xfree(1);xP = xfree(13);
yQ = xfree(3);yP = xfree(15);
zQ = xfree(5);zP = xfree(17);
vec = [xP;yP;zP]-[xQ;yQ;zQ];
dis = norm(vec,2);
end

