clc
clear
% clf
close all


do_gif = true;
filename = 'state_space_2pend.gif';  % filename to write to

%%% CONSTANTS
%---------------------------------------------------
mu = 0.7; % coefficient of friction
g = 9.81;  % (m/s^2) gravity
P.mu = mu;
P.g = g;

%%% ROBOT properties  
%---------------------------------------------------
m = 1.0;   % (kg) pendulum mass
l = 1.0;   % (m) pendulum length
tmax = 2;  % (N/m) max torque from actuator 
P.m = m;
P.l = l;
P.tmax = tmax;

%%% CONTROLLER properties 
%---------------------------------------------------
% Commanded position (must be a fixed point e.g. swing up or swing down)
P.th_command = pi/2; % Goal position (pi/2 is upright)
P.noise = 0;

% Set which control function to use
% SWING_UP uses energy shaping to get into the 
%           neighborhood and LQR to stabilize
controlfunc = @(z,P)swing_up(z,P);
P.stepsize = 0.05; % step size of euler integration

%%% SIMULATION properties
%---------------------------------------------------
% Set whether friction and control are present in sim
P.friction = true;
P.control = false;

% Initial state vector
q = [pi+0.01 ; 0];   % [ initial theta ; initial dtheta ]
q2 = [pi-0.01 ; 0];   % [ initial theta ; initial dtheta ]

q3 = [0.01 ; 0];   % [ initial theta ; initial dtheta ]

P.tSpan = [0 20];     % timespan to integrate over


%%% Set up plot
h = figure('pos',[400 300 800 400]);
set(gcf, 'Toolbar', 'none', 'Menu', 'none');
set(gcf, 'Name', 'Pendulum Simulator', 'NumberTitle', 'Off') 

tic;  %Start a timer
timeNow = 0;
len = l;

duration = P.tSpan(2)/P.stepsize;

% State space plot (dtheta vs theta)

s3 = subplot(1,1,1);
dtheta_plot = animatedline('Color','k','LineWidth',3); % fall left
dtheta_plot2 = animatedline('Color','k','LineWidth',3); % fall right
dtheta_plot3 = animatedline('Color','b','LineWidth',3); % linearized
xlabel('theta (rad)')
ylabel('dtheta (rad/s)');
xlim auto
set(s3,'xlim',1.2*[-2*pi+q(1) 2*pi+q(1)])
set(s3,'ylim',1.2*[-2*pi 2*pi])

H = m*l^2;
G = (g*m*sin(pi))/l;
C = 0;
mu = mu*0;


%     a = -(w/2 + g*l*m*sin(th))/(l^2*m)
% J(th,w) = [ df/dth df/dw ; 
%            dg/dth dg/dw ];
% J = [ 0 1 ;
%       g/l 0 ];
    
     
%     A = [ 0    1  ; 
%          -g/l  0  ];
A = [ 0   1  ;
     g/l^3  0  ];
q0 = [ pi 0 ];


if do_gif
    f = getframe(h);
    [im,map] = rgb2ind(f.cdata,10);
    im(1,1,1,20) = 0;
end

for i=1:P.tSpan(2)/P.stepsize
    ddq = pendulum_dynamics(0,q,P,controlfunc);
    ddq2 = pendulum_dynamics(0,q2,P,controlfunc);
    
    q(1) = q(1) + P.stepsize*ddq(1);
    q(2) = q(2) + P.stepsize*ddq(2);

    q2(1) = q2(1) + P.stepsize*ddq2(1);
    q2(2) = q2(2) + P.stepsize*ddq2(2);
    
    
    
    q3 = q3 + P.stepsize*(A*q3);

    
    th = q(1);
    dth = q(2);
    th2 = q2(1);
    dth2 = q2(2);
    th3 = q3(1);
    dth3 = q3(2);

    
    % Update plots
    % If we selected no control, plot 0 torque
    if P.control
        tau = controlfunc([th;dth], P);  % calculate torque using controller
        tau2 = controlfunc([th2;dth2], P);  % calculate torque using controller
        tau3 = controlfunc([th3;dth3], P);  % calculate torque using controller
    else
        tau = 0;
    end
    addpoints(dtheta_plot,th,dth)       % add point to state space curve
    addpoints(dtheta_plot2,th2,dth2)       % add point to state space curve
%     addpoints(dtheta_plot3,th3,dth3)       % add point to state space curve

    drawnow update
    pause(0.007);
    timeNow = toc;
    if do_gif
        f = getframe(h);  
        im(:,:,1,i) = rgb2ind(f.cdata,map);
%     im(:,:,1,k) = rgb2ind(f.cdata,map,'nodither');
    end
end

if do_gif
    imwrite(im,map,filename,'DelayTime',0.001,'LoopCount',inf) %g443800
end
% hold on
% x = -20:20;
% [V,D] = eig(A)
% y1 = V(2,1)/V(1,1) * (x-pi);
% y2 = V(2,2)/V(1,2) * (x-pi);
% plot(x,y1,'b','LineWidth',3)
% plot(x,y2,'r','LineWidth',3)







