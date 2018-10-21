function ddq = pendulum_dynamics(~,q,P,controller)

m = P.m;
g = P.g;
l = P.l;

% % UNPACK STATE
theta = q(1,:);
dtheta = q(2,:);

% call to controller
tau = controller(q,P);

% assemble G value for manip equations
H = m*l^2;
G = (g*sin(theta))/l;
mu = P.mu*dtheta;

%%% DYNAMICS properties 
%---------------------------------------------------
% Chooses how to simulate depending on P values set in pendulum 
% SIMULATION properties section in pendulum.m

if  P.friction && P.control
    ddq = (-G - tau - mu)/H;   % dynamics + controller + friction
    
elseif  P.friction 
    ddq = (-G - mu)/H;          % dynamics + friction
    
elseif  P.control
    ddq = (-G - tau)/H;         % dynamics + controller
    
else
    ddq = (-G)/H;               % dynamics 
    
end

% First derivative of state vector with second derivative of state vector
ddq = [dtheta; ddq];

end