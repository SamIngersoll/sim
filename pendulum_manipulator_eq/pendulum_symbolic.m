clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                        %
%       Create Symbolic Variables        %
%                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms th w a;        % position, velocity, acceleration
syms m g l tau;     % mass, gravity, length, torque
q =  [ th w ];      % state
dq = [ w  a ];      % derivative of state

T = (1/2)*m*(l^2)*(w^2);    % kinetic energy of the pendulum
U = m*g*l*(1-cos(th));      % potential energy of the pendulum
L = T - U;  % lagrangian

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                           %
%       Evaluate Partial Derivatives        %
%                                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

dL_dth = jacobian(L,th);
dL_dw = jacobian(L,w);
% Note the application of the chain rule:
ddL_dtdw = jacobian(dL_dw,q) * dq.';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              %
%       Solve Equations        %
%                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

mu = w*.5;  % friction

% Equations of motion:
eqn = ddL_dtdw - dL_dth + mu - tau
% eqn = ddL_dtdw - dL_dth

