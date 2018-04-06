function [ Ften_proj] = LG_isTension(u,xtautdot,x )
%LG_ISNOTENSION Summary of this function goes here
%% input force
Fzb = u(1) + 0.6*9.81;
phi = x(11);theta = x(13);psi = x(15);
R = euler2rotMat(phi, theta, psi);
Fzg = R*[0;0;Fzb];
%% virtual inertial force:
% Quadcopter Parameters:
mQ=0.55;g=9.8;
%   Detailed explanation goes here
dxQdot = xtautdot(2);
dyQdot = xtautdot(4);
dzQdot = xtautdot(6);
Facc = -mQ*[dxQdot;dyQdot;dzQdot];% virtual inertial force:
%% mass 
mQg = mQ*9.8*[0;0;-1];
%% calculate tension: Ften + mQg + Fzg + Facc = 0;
Ften = -(mQg+Fzg+Facc);
%% project Ften into vecQ2P:
xQ = x(1);yQ = x(3);zQ = x(5);
xP = x(17);yP = x(19);zP = x(21);
posP = [xP;yP;zP];posQ = [xQ;yQ;zQ];
vecQ2P = (posP - posQ)/norm((posP - posQ),2);% Q2P的单位向量
Ften_proj = dot(Ften,vecQ2P);
% %% vec: Q2P
% 
% vP = [dxP;dyP;dzP];
% 
% vQ = [dxQ;dyQ;dzQ];
% 
% %% acc of P paraller and tangent with respect to vecP2Q
% accP_proj = dot(accP,vecP2Q);
% ag_proj = dot(ag,vecP2Q);
% %accP_tan = accP - accP_proj*vecP2Q;
% %%
% vP_proj = dot(vP,vecP2Q);
% vP_tan = vP - vP_proj*vecP2Q; % vP在垂直于P2Q平面上的分量
% 
% vQ_proj = dot(vQ,vecP2Q);
% vQ_tan = vQ - vQ_proj*vecP2Q;
% 
% error = (vP_proj-vQ_proj);
% % error = dot(vP_tan,vP_tan);
% % F + Fn + Fi = 0;
% error1 = ag_proj;
% error2 = -accP_proj; % 惯性力为实际加速度反方向
% error = -(error1 + error2); % 张力加速度
if(Ften_proj>0.0001)
    is = 1; % no tension
else
    is = 0;
end
end

function R = euler2rotMat(phi, theta, psi)
    R(1,1,:) = cos(psi).*cos(theta);
    R(1,2,:) = -sin(psi).*cos(phi) + cos(psi).*sin(theta).*sin(phi);
    R(1,3,:) = sin(psi).*sin(phi) + cos(psi).*sin(theta).*cos(phi);
    
    R(2,1,:) = sin(psi).*cos(theta);
    R(2,2,:) = cos(psi).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    R(2,3,:) = -cos(psi).*sin(phi) + sin(psi).*sin(theta).*cos(phi);
    
    R(3,1,:) = -sin(theta);
    R(3,2,:) = cos(theta).*sin(phi);
    R(3,3,:) = cos(theta).*cos(phi);
end
