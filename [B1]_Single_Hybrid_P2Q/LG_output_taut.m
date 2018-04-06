function [ xoutput] = LG_output_taut( xtaut)
%LG_OUT_N Summary of this function goes here
% xQ_dec  = xdec(1);
% dxQ_dec = xdec(2);
% yQ_dec  = xdec(3);
% dyQ_dec = xdec(4);
% zQ_dec  = xdec(5);
% dzQ_dec = xdec(6);
% alpha_dec  = xdec(7);
% dalpha_dec = xdec(8);
% beta_dec   = xdec(9);
% dbeta_dec  = xdec(10);
% phi_dec    = xdec(11);
% dphi_dec   = xdec(12);
% theta_dec  = xdec(13);
% dtheta_dec = xdec(14);
% psi_dec    = xdec(15);
% dpsi_dec   = xdec(16);
% xP_dec  = xdec(17);
% dxP_dec = xdec(18);
% yP_dec  = xdec(19);
% dyP_dec = xdec(20);
% zP_dec  = xdec(21);
% dzP_dec = xdec(22);
% taut mode: generate all states based on 16 states.
%   Detailed explanation goes here
Lc = 0.5;
% x = [xtaut;zeros(6,1)];
%% there is no problem in terms of PosP
xQ = xtaut(1);dxQ = xtaut(2);
yQ = xtaut(3);dyQ = xtaut(4);
zQ = xtaut(5);dzQ = xtaut(6);
%% the only problem is to define/get PosP and VelP
%  discuss in two different cases:
% isclose = LG_iscloseEm( xdec );
% if(isclose>0.0001)% case 1: system will jump into free mode,  
%     xP  = xdec(17);
%     dxP = xdec(18);
%     yP  = xdec(19);
%     dyP = xdec(20);
%     zP  = xdec(21);
%     dzP = xdec(22);
% else% case 2: system will stay in taut mode, 
%     alpha = xtaut(7);dalpha = xtaut(8);
%     beta = xtaut(9);dbeta = xtaut(10);
% 
%     xP = xQ + Lc*sin(beta);
%     dxP = dxQ + Lc*cos(beta)*dbeta;
%     yP = yQ + Lc*sin(alpha).*cos(beta);
%     dyP = dyQ + Lc*(cos(alpha).*dalpha).*cos(beta)...
%         - Lc*sin(alpha).*(sin(beta)*dbeta);
%     zP = zQ - Lc*cos(alpha).*cos(beta);
%     dzP = dzQ + Lc*sin(beta)*dbeta*cos(alpha)...
%         + Lc*cos(beta)*sin(alpha)*dalpha;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    alpha = xtaut(7);dalpha = xtaut(8);
    beta = xtaut(9);dbeta = xtaut(10);

    xP = xQ + Lc*sin(beta);
    dxP = dxQ + Lc*cos(beta)*dbeta;
    yP = yQ + Lc*sin(alpha).*cos(beta);
    dyP = dyQ + Lc*(cos(alpha).*dalpha).*cos(beta)...
        - Lc*sin(alpha).*(sin(beta)*dbeta);
    zP = zQ - Lc*cos(alpha).*cos(beta);
    dzP = dzQ + Lc*sin(beta)*dbeta*cos(alpha)...
        + Lc*cos(beta)*sin(alpha)*dalpha;
%% update xoutput
xoutput = [xtaut;zeros(6,1)];
xoutput(17) = xP;
xoutput(18) = dxP;
xoutput(19) = yP;
xoutput(20) = dyP;
xoutput(21) = zP;
xoutput(22) = dzP;
end
% 
% function [ isclose ] = LG_iscloseEm( x )
% %LG_DIS Summary of this function goes here
% %   Detailed explanation goes here
% xQ  = x(1);
% dxQ = x(2);
% yQ  = x(3);
% dyQ = x(4);
% zQ  = x(5);
% dzQ = x(6);
% alpha  = x(7);
% dalpha = x(8);
% beta   = x(9);
% dbeta  = x(10);
% phi    = x(11);
% dphi   = x(12);
% theta  = x(13);
% dtheta = x(14);
% psi    = x(15);
% dpsi   = x(16);
% xP  = x(17);
% dxP = x(18);
% yP  = x(19);
% dyP = x(20);
% zP  = x(21);
% dzP = x(22);
% vecQ = [dxQ;dyQ;dzQ];% vector of Q
% vecP = [dxP;dyP;dzP];% vector of P
% rhoQ2P = [sin(beta);cos(beta)*sin(alpha);-cos(beta)*cos(alpha)];
% vecQ_proj = dot(vecQ,rhoQ2P)*rhoQ2P;% projection of VecQ on cable(Q2P)
% vecP_proj = dot(vecP,rhoQ2P)*rhoQ2P; % projection of VecP on cable(Q2P)
% isclose = dot((vecQ_proj - vecP_proj),rhoQ2P);
% end
