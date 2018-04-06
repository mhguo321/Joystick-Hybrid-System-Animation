function xdot = LG_f_taut(u,xtaut)
%% taut mode: 6DoF (quadrotor) + 2DoF (swing) = 16 states
xdot = zeros(16,1);
%%  6 states for quadrotor position
xQ = xtaut(1);dxQ = xtaut(2);
yQ = xtaut(3);dyQ = xtaut(4);
zQ = xtaut(5);dzQ = xtaut(6);
%% 2 states for swing 
alpha = xtaut(7);dalpha = xtaut(8);
beta = xtaut(9);dbeta = xtaut(10);
%% 6 states for quadrotor attitude
phi = xtaut(11);dphi = xtaut(12);
theta = xtaut(13);dtheta = xtaut(14);
psi = xtaut(15);dpsi = xtaut(16);
%% control inputs:
Fz = u(1) + 0.6*9.81;
Mx = u(2);
My = u(3);
Mz = u(4);
%%
% Parameters:
% Quadcopter Parameters:
mQ=0.55;g=9.81;
% LQ = 0.17;
% kf = 2.98e-6;
% kM = 1.14e-7;

Ix=0.0023;
Iy=0.0028;
Iz=0.0046;

% Payload parameters:
mP = 0.05;
Lr = 0.5;
%% Generalized Force 
M = [mP+mQ,0,0,0,Lr.*mP.*cos(beta),0,0,0;0,mP+mQ,0,Lr.*mP.*cos(alpha) ...
  .*cos(beta),(-1).*Lr.*mP.*sin(alpha).*sin(beta),0,0,0;0,0,mP+mQ, ...
  Lr.*mP.*cos(beta).*sin(alpha),Lr.*mP.*cos(alpha).*sin(beta),0,0,0; ...
  0,Lr.*mP.*cos(alpha).*cos(beta),Lr.*mP.*cos(beta).*sin(alpha), ...
  Lr.^2.*mP.*cos(beta).^2,0,0,0,0;Lr.*mP.*cos(beta),(-1).*Lr.*mP.* ...
  sin(alpha).*sin(beta),Lr.*mP.*cos(alpha).*sin(beta),0,Lr.^2.*mP,0, ...
  0,0;0,0,0,0,0,Ix,0,(-1).*Ix.*sin(theta);0,0,0,0,0,0,Iy.*cos(phi) ...
  .^2+Iz.*sin(phi).^2,(Iy+(-1).*Iz).*cos(phi).*cos(theta).*sin(phi); ...
  0,0,0,0,0,(-1).*Ix.*sin(theta),(Iy+(-1).*Iz).*cos(phi).*cos(theta) ...
  .*sin(phi),cos(theta).^2.*(Iz.*cos(phi).^2+Iy.*sin(phi).^2)+Ix.* ...
  sin(theta).^2];

fdq1 = dbeta.^2.*Lr.*mP.*sin(beta)+Fz.*sin(phi).*sin(psi)+Fz.*cos(phi).* ...
  cos(psi).*sin(theta);

fdq2 = (dalpha.^2+dbeta.^2).*Lr.*mP.*cos(beta).*sin(alpha)+2.*dalpha.* ...
  dbeta.*Lr.*mP.*cos(alpha).*sin(beta)+(-1).*Fz.*cos(psi).*sin(phi)+ ...
  Fz.*cos(phi).*sin(psi).*sin(theta);

fdq3 = (-1).*g.*(mP+mQ)+(-1).*(dalpha.^2+dbeta.^2).*Lr.*mP.*cos(alpha).* ...
  cos(beta)+Fz.*cos(phi).*cos(theta)+2.*dalpha.*dbeta.*Lr.*mP.*sin( ...
  alpha).*sin(beta);

fdq4 = Lr.*mP.*cos(beta).*((-1).*g.*sin(alpha)+2.*dalpha.*dbeta.*Lr.*sin( ...
  beta));

fdq5 = (-1).*Lr.*mP.*(g.*cos(alpha)+dalpha.^2.*Lr.*cos(beta)).*sin(beta);

fdq6 = Mx+dpsi.*dtheta.*(Ix+(Iy+(-1).*Iz).*cos(2.*phi)).*cos(theta)+(-1) ...
  .*(Iy+(-1).*Iz).*cos(phi).*(dtheta.^2+(-1).*dpsi.^2.*cos(theta) ...
  .^2).*sin(phi);

fdq7 = cos(phi).*(My+2.*dphi.*dtheta.*(Iy+(-1).*Iz).*sin(phi))+(1/4).*(( ...
  -4).*Mz.*sin(phi)+2.*dpsi.*(Iy+(-1).*Iz).*cos(2.*phi).*cos(theta) ...
  .*((-2).*dphi+dpsi.*sin(theta))+(-2).*dpsi.*cos(theta).*(2.*dphi.* ...
  Ix+dpsi.*((-2).*Ix+Iy+Iz).*sin(theta)));

fdq8 = dphi.*dpsi.*((-1).*Iy+Iz).*cos(theta).^2.*sin(2.*phi)+(-1).*(Mx+ ...
  dtheta.^2.*((-1).*Iy+Iz).*cos(phi).*sin(phi)).*sin(theta)+cos( ...
  theta).*(dphi.*dtheta.*Ix+Mz.*cos(phi)+My.*sin(phi)+dpsi.*dtheta.* ...
  ((-2).*Ix+Iy+Iz).*sin(theta)+(-1).*dtheta.*(Iy+(-1).*Iz).*cos(2.* ...
  phi).*(dphi+dpsi.*sin(theta)));

fdq = [fdq1;fdq2;fdq3;fdq4;fdq5;fdq6;fdq7;fdq8];

dummy = M\fdq;

xQdot = dxQ;
dxQdot = dummy(1);
yQdot = dyQ;
dyQdot = dummy(2);
zQdot = dzQ;
dzQdot = dummy(3);

alphadot = dalpha;
dalphadot = dummy(4);
betadot = dbeta;
dbetadot = dummy(5);

phidot = dphi;
dphidot = dummy(6);
thetadot = dtheta;
dthetadot = dummy(7);
psidot = dpsi;
dpsidot = dummy(8);
%%
xdot(1) = xQdot;
xdot(2) = dxQdot;
xdot(3) = yQdot;
xdot(4) = dyQdot;
xdot(5) = zQdot;
xdot(6) = dzQdot;

xdot(7) = alphadot;
xdot(8) = dalphadot;
xdot(9) = betadot;
xdot(10) = dbetadot;

xdot(11) = phidot;
xdot(12) = dphidot;
xdot(13) = thetadot;
xdot(14) = dthetadot;
xdot(15) = psidot;
xdot(16) = dpsidot;
end
