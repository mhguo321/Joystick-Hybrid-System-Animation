function [x] = LG_initial( varargin )
%LG_INITIAL Summary of this function goes here:
% there are two cases to initial the hybrid system:
% case 1: taut mode, there are 5 inputs as follows:
    % varargin{1} = xQ; varargin{2} = yQ; varargin{3} = zQ;
    % varargin{4} = alpha[deg]; varargin{5} = beta[deg];
    
% case 2: free mode, thera are 6 inputs as follows:
    % varargin{1} = xQ; varargin{2} = yQ; varargin{3} = zQ;
    % varargin{4} = xQ; varargin{5} = yQ; varargin{6} = zQ;
    
%   Detailed explanation goes here
Lc = 0.5;
x = zeros(22,1);
%% specify Quadrotor position
% x(1) = varargin{1}; % xQ
% x(3) = varargin{2}; % yQ
% x(5) = varargin{3}; % zQ
xQ = varargin{1}; % xQ
yQ = varargin{2}; % yQ
zQ = varargin{3}; % zQ
x(1) = xQ; % xQ
x(3) = yQ; % yQ
x(5) = zQ; % zQ
%% specify Quadrotor position in two cases:
if nargin == 5 % taut mode: 
    alpha0 = deg2rad(varargin{4});
    beta0 = deg2rad(varargin{5});
    x(7) = alpha0;
    x(9) = beta0;
    % vector from Q to P
    posQ = [x(1);x(3);x(5)];
    rho = [sin(beta0);cos(beta0)*sin(alpha0);-cos(beta0)*cos(alpha0)];
    temp = posQ + rho*Lc;
    x(17) = temp(1); % xQ
    x(19) = temp(2); % yQ
    x(21) = temp(3); % zQ
end
if nargin == 6 % free mode:
        xP = varargin{4};% xP
        yP = varargin{5};% yP
        zP = varargin{6};% zP
        
        posQ = [xQ;yQ;zQ];
        posP = [xP;yP;zP];
        dis = norm(posQ-posP,2); % distance between Q and P
        
        if(dis>Lc)
            zP = -sqrt(Lc^2 - xP^2 - yP^2);
        end
        
        x(17) = xP;% xP
        x(18) = 0;%dxP
        x(19) = yP;% yP
        x(20) = 0;%dyP
        x(21) = zP;% zP
        x(22) = 0;%dzP
end
end

