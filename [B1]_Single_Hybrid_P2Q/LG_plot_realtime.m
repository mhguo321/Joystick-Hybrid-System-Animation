function [sys,x0,str,ts] = LG_plot_realtime(t,x,u,flag) 
%%
 switch flag
    case 0
    [sys,x0,str,ts]=mdlInitializeSizes(); 
    case 1
    sys=mdlDerivatives(t,x,u); 
    case 3
    sys=mdlOutputs(t,x,u); 
    case { 2, 4, 9 } 
    sys = []; 
    otherwise 
    error(['Unhandled flag = ',num2str(flag)]); 
end 
function [sys,x0,str,ts]=mdlInitializeSizes()
%% 
sizes = simsizes; 
sizes.NumContStates = 0; 
sizes.NumDiscStates = 0; 
sizes.NumOutputs = 0; 
sizes.NumInputs = 10; 
sizes.DirFeedthrough = 0; 
sizes.NumSampleTimes = 1; 
sys = simsizes(sizes); 
  
x0 = [ ];
str = []; 
ts = [0 0]; 
function sys=mdlDerivatives(t,x,u) 
%% Model states and controllors:
sys = []; 
function sys=mdlOutputs(t,x,u)
% fig = figure('Name', '6DOF Animation(Email:gmh_njust@163.com)');
% set(gca, 'drawmode', 'fast');
x = u(1);
y = u(2);
coder.extrinsic('plot')
plot(x,y,'s','Markersize',8,'MarkerFaceColor','g','erasemode','background')
set(gca,'XLim',[0 12],'Ylim',[-1 1]);
sys = [];

function sys=mdlGetTimeOfNextVarHit(t,x,u)
sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

function sys=mdlTerminate(t,x,u)
sys = [];

