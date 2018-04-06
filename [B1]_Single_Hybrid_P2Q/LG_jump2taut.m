function [ xtaut,xout ] = LG_jump2taut( x )
%LG_Z2N Summary of this function goes here
%  xout: all states after jumping into tatu mode
%  xtaut: taut mode states after jumping

%   states jump from free mode into taut mode:
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
%%
Lc = 0.5;
mQ = 0.55;mP = 0.05;% mass 
vecQ = [dxQ;dyQ;dzQ];% vector of Q before jump
vecP = [dxP;dyP;dzP];% vector of P before jump
rhoQ2P = [sin(beta);cos(beta)*sin(alpha);-cos(beta)*cos(alpha)];
% direction vector from Q to P
vecQ_proj = dot(vecQ,rhoQ2P)*rhoQ2P;% projection of VecQ on cable(Q2P)
vecQ_left = vecQ - vecQ_proj;% VecQ normal to cable
vecP_proj = dot(vecP,rhoQ2P)*rhoQ2P; % projection of VecP on cable(Q2P)
vecP_left = vecP - vecP_proj; % VecP normal to cable
% after jumping:
%% case 1： 完全非弹性碰撞（碰撞后，速度相同，理解为粘在一起）
% 动量守恒，能量不守恒，速度一致：
% vec_proj_after = (mQ*vecQ_proj + mP*vecP_proj)/(mP+mQ);
% vecQ_after = vec_proj_after + vecQ_left;
% vecP_after = vec_proj_after + vecP_left;
%% case 2: 完全弹性碰撞：动量守恒，能量守恒：碰撞后，各自有速度
% vecQ_proj_after = ( (mQ-mP)*vecQ_proj + 2*mP*vecP_proj )/(mQ+mP);
% vecQ_after = vecQ_proj_after + vecQ_left;
% vecP_proj_after = ( (mP-mQ)*vecP_proj + 2*mQ*vecQ_proj )/(mQ+mP);
% vecP_after = vecP_proj_after + vecP_left;
%% case 3: 非弹性碰撞，前两者的综合，利用恢复系数k来调节：
% k = 0: case 1: 机械能不守恒
% k = 1; case 2: 机械能守恒
k = 0;
vecQ_proj_after = vecQ_proj - (1+k)*(vecQ_proj-vecP_proj)*mP/(mQ+mP);
vecQ_after = vecQ_proj_after + vecQ_left;
vecP_proj_after = vecP_proj + (1+k)*(vecQ_proj-vecP_proj)*mQ/(mQ+mP);
vecP_after = vecP_proj_after + vecP_left;
%%
dxQ_after = vecQ_after(1);%dxQ
dyQ_after = vecQ_after(2);%dyQ
dzQ_after = vecQ_after(3);%dzQ

dxP_after = vecP_after(1);%dxP
dyP_after = vecP_after(2);%dyP
dzP_after = vecP_after(3);%dzP

beta = asin((xP-xQ)/Lc);
alpha = asin((yP-yQ)/Lc/cos(beta));
dbeta = (dxP_after-dxQ_after)/(Lc*cos(beta));
dalpha = (dyP_after-dyQ_after+Lc*sin(beta)*sin(alpha)*dbeta)/(Lc*cos(beta)*cos(alpha));
%%
% update x
xout = x;
xout(2) = vecQ_after(1);%dxQ
xout(4) = vecQ_after(2);%dyQ
xout(6) = vecQ_after(3);%dzQ

xout(18) = dxP_after;%dxP
xout(20) = dyP_after;%dyP
xout(22) = dzP_after;%dzP
% update xtaut:
xtaut = xout(1:16);
xtaut(2)  = dxQ_after;%dxQ
xtaut(4)  = dyQ_after;%dyQ
xtaut(6)  = dzQ_after;%dzQ
xtaut(7) = alpha;
xtaut(8)  = dalpha;
xtaut(9) = beta;
xtaut(10) = dbeta;
end

