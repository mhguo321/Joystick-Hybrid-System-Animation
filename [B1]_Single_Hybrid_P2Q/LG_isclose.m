function [ isclose ] = LG_isclose( x )
%LG_DIS Summary of this function goes here
%   Detailed explanation goes here
xQ  = x(1);
dxQ = x(2);
yQ  = x(3);
dyQ = x(4);
zQ  = x(5);
dzQ = x(6);
alpha  = x(7);
dalpha = x(8);
beta   = x(9);
dbeta  = x(10);
phi    = x(11);
dphi   = x(12);
theta  = x(13);
dtheta = x(14);
psi    = x(15);
dpsi   = x(16);
xP  = x(17);
dxP = x(18);
yP  = x(19);
dyP = x(20);
zP  = x(21);
dzP = x(22);
vecQ = [dxQ;dyQ;dzQ];% vector of Q
vecP = [dxP;dyP;dzP];% vector of P
posQ = [xQ;yQ;zQ];
posP = [xP;yP;zP];
Lc = 0.5;
dis = norm(posQ-posP,2); % distance between Q and P
rhoQ2P = [sin(beta);cos(beta)*sin(alpha);-cos(beta)*cos(alpha)];
vecQ_proj = dot(vecQ,rhoQ2P)*rhoQ2P;% projection of VecQ on cable(Q2P)
vecP_proj = dot(vecP,rhoQ2P)*rhoQ2P; % projection of VecP on cable(Q2P)
vecQ2P = dot((vecQ_proj - vecP_proj),rhoQ2P);
if((dis<0.49)||(vecQ2P>0)) %
    isclose = 1;
else
    isclose = -1;
end
tem = 3;
end

