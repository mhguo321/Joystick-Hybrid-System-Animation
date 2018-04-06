function xdot_output = LG_f_free(u,xfree)
%% free mode: 6DoF (quadrotor) + 3DoF (load) = 18 states
%#codegen
xdot = zeros(18,1);
%%  12 states for quadrotor: 
xQ = xfree(1);dxQ = xfree(2);
yQ = xfree(3);dyQ = xfree(4);
zQ = xfree(5);dzQ = xfree(6);

phi = xfree(7);dphi = xfree(8);
theta = xfree(9);dtheta = xfree(10);
psi = xfree(11);dpsi = xfree(12);
%%  6 states for payload:
xP = xfree(13);dxP = xfree(14);
yP = xfree(15);dyP = xfree(16);
zP = xfree(17);dzP = xfree(18);
%%
posQ = [xQ;yQ;zQ];
posP = [xP;yP;zP];
dis = norm(posQ-posP,2); % distance between Q and P
if(dis>=0.5)
    Stop = 0;
else
    Stop = 1;
end
%% control inputs: 
Ft = u(1) + 0.55 * 9.81;
Mx = u(2);
My = u(3);
Mz = u(4);
%%
% Parameters:
% Quadcopter Parameters:
mQ=0.55;g=9.81;
% LQ = 0.17;
% kF = 2.98e-6;
% kM = 1.14e-7;

Ixx=0.0023;
Iyy=0.0028;
Izz=0.0046;

% Payload parameters:
mP = 0.05;
Lr = 0.5;
%% Model 1: for free payload:
xPdot = dxP;
dxPdot = 0;
yPdot = dyP;
dyPdot = 0;
zPdot = dzP;
dzPdot = -g;
%% Model 2: for quadcopter:
M = [mQ,0,0,0,0,0;0,mQ,0,0,0,0;0,0,mQ,0,0,0;0,0,0,Ixx,0,(-1).*Ixx.* ...
  sin(theta);0,0,0,0,Iyy.*cos(phi).^2+Izz.*sin(phi).^2,(Iyy+(-1).* ...
  Izz).*cos(phi).*cos(theta).*sin(phi);0,0,0,(-1).*Ixx.*sin(theta),( ...
  Iyy+(-1).*Izz).*cos(phi).*cos(theta).*sin(phi),Izz.*cos(phi).^2.* ...
  cos(theta).^2+Iyy.*cos(theta).^2.*sin(phi).^2+Ixx.*sin(theta).^2]; 

fdq1 = Ft.*(sin(phi).*sin(psi)+cos(phi).*cos(psi).*sin(theta));

fdq2 = Ft.*((-1).*cos(psi).*sin(phi)+cos(phi).*sin(psi).*sin(theta));

fdq3 = (-1).*g.*mQ+Ft.*cos(phi).*cos(theta);

fdq4 = Mx+dpsi.*dtheta.*(Ixx+(Iyy+(-1).*Izz).*cos(2.*phi)).*cos(theta)+( ...
  -1).*(Iyy+(-1).*Izz).*cos(phi).*(dtheta.^2+(-1).*dpsi.^2.*cos( ...
  theta).^2).*sin(phi);

fdq5 = cos(phi).*(My+2.*dphi.*dtheta.*(Iyy+(-1).*Izz).*sin(phi))+(1/2).*( ...
  (-2).*Mz.*sin(phi)+dpsi.*cos(theta).*((-1).*dpsi.*(Iyy+Izz).*sin( ...
  theta)+(-2).*Ixx.*(dphi+(-1).*dpsi.*sin(theta))+(Iyy+(-1).*Izz).* ...
  cos(2.*phi).*((-2).*dphi+dpsi.*sin(theta))));

fdq6 = dphi.*dpsi.*((-1).*Iyy+Izz).*cos(theta).^2.*sin(2.*phi)+(-1).*(Mx+ ...
  dtheta.^2.*((-1).*Iyy+Izz).*cos(phi).*sin(phi)).*sin(theta)+cos( ...
  theta).*(dphi.*dtheta.*Ixx+Mz.*cos(phi)+My.*sin(phi)+dpsi.* ...
  dtheta.*((-2).*Ixx+Iyy+Izz).*sin(theta)+(-1).*dtheta.*(Iyy+(-1).* ...
  Izz).*cos(2.*phi).*(dphi+dpsi.*sin(theta)));

fdq = [fdq1;fdq2;fdq3;fdq4;fdq5;fdq6];

dummy = M\fdq;
xQdot = dxQ;
dxQdot = dummy(1);
yQdot = dyQ;
dyQdot = dummy(2);
zQdot = dzQ;
dzQdot = dummy(3);

phidot = dphi;
dphidot = dummy(4);
thetadot = dtheta;
dthetadot = dummy(5);
psidot = dpsi;
dpsidot = dummy(6);

%%
xdot(1) = xQdot;
xdot(2) = dxQdot;
xdot(3) = yQdot;
xdot(4) = dyQdot;
xdot(5) = zQdot;
xdot(6) = dzQdot;

xdot(7) = phidot;
xdot(8) = dphidot;
xdot(9) = thetadot;
xdot(10) = dthetadot;
xdot(11) = psidot;
xdot(12) = dpsidot;

xdot(13) = xPdot;
xdot(14) = dxPdot;
xdot(15) = yPdot;
xdot(16) = dyPdot;
xdot(17) = zPdot;
xdot(18) = dzPdot;
xdot_output = xdot;
end
