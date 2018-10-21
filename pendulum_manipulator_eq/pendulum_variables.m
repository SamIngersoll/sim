function P = pendulum_variables()

%---------------------------------------------------
% CONSTANTS
%---------------------------------------------------
P.mu = 0.1; % coefficient of friction
P.g = 9.81;  % (m/s^2) gravity


%---------------------------------------------------
% ROBOT properties
%---------------------------------------------------
P.m = 1.0;   % (kg) pendulum mass
P.l = 1.0;   % (m) pendulum length
P.tmax = 2;  % (N/m) max torque from actuator 


%---------------------------------------------------
% CONTROLLER properties
%---------------------------------------------------
% Commanded position (must be a fixed point e.g. swing up or swing down)
P.th_command = pi/2; % Goal position (pi/2 is upright)
P.noise = 0;

% Set which control function to use
% SWING_UP uses energy shaping to get into the 
%           neighborhood and LQR to stabilize
P.controlfunc = @(z,P)swing_up(z,P);
P.stepsize = 0.008; % step size of euler integration


%---------------------------------------------------
% SIMULATION properties
%---------------------------------------------------
% Set whether friction and control are present in sim
P.friction = true;
P.control = true;

% Initial state vector
P.q0 = [0.01 ; 0];   % [ initial theta ; initial dtheta ]
P.tSpan = [0 7];     % timespan to integrate over


%---------------------------------------------------
% GIF
%---------------------------------------------------
P.do_gif = false;
P.filename = 'pendulum.gif';  % filename to write to